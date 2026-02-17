/**
 * SMART HYDRAULIC CONTROL SYSTEM (FULL SIMULATION CONTROL)
 * --------------------------------------------------------
 * - Inputs: Toggle between Live Sensors (Type T) or Slider Simulation
 * - Outputs: Toggle between Live Relays or LED-Only Simulation
 * - Hardware: LabJack T7 + DSD Tech Relays
 */

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#include <iostream>
#include <vector>
#include <string>
#include <atomic>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <mutex>
#include "httplib.h"
#include <LabJackM.h> 

using namespace std;

// --- CONFIGURATION CONSTANTS ---
const double WATER_CP = 4186.0; 

// --- GLOBAL SHARED STATE ---
struct SystemState {
    // Logic Inputs
    atomic<double> t1{20.0}; 
    atomic<double> t2{25.0}; 
    atomic<double> t3{55.0}; 
    atomic<double> t4{25.0}; 

    // Raw Real Sensor Data
    atomic<double> realSensors[6]; 

    // Simulation Overrides
    atomic<double> simSensors[4]; 
    atomic<bool>   useSensorSimulation{true}; // Input Toggle
    atomic<bool>   useRelaySimulation{true};  // Output Toggle (NEW)

    // User Config
    atomic<double> targetWarm{40.0};
    atomic<double> targetHot{90.0};
    atomic<double> massFlowRateL_min{1.0};
    atomic<double> heaterMaxPowerW{3000.0};
    atomic<int>    preTimeSec{5};
    atomic<int>    postTimeSec{5};
    atomic<double> tankTargetTemp{65.0}; 
    atomic<double> tankMaxDelta{2.0};    

    // System Status
    atomic<double> calculatedPower{0.0};
    atomic<double> heaterDutyCycle{0.0};
    string currentMode = "AUTO";         
    string currentStage = "IDLE";        
    string statusMessage = "Initializing...";

    // Relays (Logic State for Dashboard)
    bool ev[7] = {0}; 
    bool pump = false;
    bool heater = false;

    // Flags
    atomic<bool> isPouring{false}; 
    mutex stateMutex;
};

SystemState SYS;

// --- 1. SERIAL PORT ---
class SerialPort {
    HANDLE hSerial;
public:
    bool connected;
    SerialPort(string portName) {
        string fullPortName = "\\\\.\\" + portName;
        hSerial = CreateFileA(fullPortName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
        connected = (hSerial != INVALID_HANDLE_VALUE);
        if(connected) {
             DCB dcb = {0}; dcb.DCBlength = sizeof(dcb); GetCommState(hSerial, &dcb);
             dcb.BaudRate = 9600; dcb.ByteSize = 8; dcb.StopBits = ONESTOPBIT; dcb.Parity = NOPARITY;
             SetCommState(hSerial, &dcb);
        }
    }
    void write(const vector<unsigned char>& data) {
        if (!connected) return;
        DWORD bytes; WriteFile(hSerial, data.data(), data.size(), &bytes, NULL);
    }
    ~SerialPort() { if(connected) CloseHandle(hSerial); }
};

// --- 2. LABJACK SENSOR MANAGER ---
class SensorManager {
    int handle = 0;
    bool connected = false;
    const char* readNames[6] = {
        "AIN48_EF_READ_A", "AIN49_EF_READ_A", "AIN50_EF_READ_A",
        "AIN51_EF_READ_A", "AIN52_EF_READ_A", "AIN53_EF_READ_A"
    };
    double readValues[6] = {0};
    int errAddress = -1;

public:
    SensorManager() {
        int err = LJM_Open(LJM_dtT7, LJM_ctANY, "ANY", &handle);
        if (err != 0) {
            cout << "[LJM] Error: Hardware not found. Running in SIMULATION only." << endl;
            return;
        }
        connected = true;
        
        const int NUM_FRAMES = 18;
        const char *configNames[NUM_FRAMES] = {
            "AIN48_NEGATIVE_CH", "AIN49_NEGATIVE_CH", "AIN50_NEGATIVE_CH", 
            "AIN51_NEGATIVE_CH", "AIN52_NEGATIVE_CH", "AIN53_NEGATIVE_CH",
            "AIN48_EF_INDEX", "AIN49_EF_INDEX", "AIN50_EF_INDEX", 
            "AIN51_EF_INDEX", "AIN52_EF_INDEX", "AIN53_EF_INDEX",
            "AIN48_EF_CONFIG_A", "AIN49_EF_CONFIG_A", "AIN50_EF_CONFIG_A", 
            "AIN51_EF_CONFIG_A", "AIN52_EF_CONFIG_A", "AIN53_EF_CONFIG_A"
        };
        double configValues[NUM_FRAMES] = {
            56, 57, 58, 59, 60, 61,   
            24, 24, 24, 24, 24, 24,   // Type T
             1,  1,  1,  1,  1,  1    
        };
        LJM_eWriteNames(handle, NUM_FRAMES, configNames, configValues, &errAddress);
    }

    void read() {
        if(!connected) return;
        int err = LJM_eReadNames(handle, 6, readNames, readValues, &errAddress);
        if(err == 0) {
            for(int i=0; i<6; i++) {
                if(readValues[i] > -200.0 && readValues[i] < 1200.0) {
                    SYS.realSensors[i] = readValues[i];
                } else {
                    SYS.realSensors[i] = -999.0; // NC
                }
            }
        }
    }
    ~SensorManager() { if(connected) LJM_Close(handle); }
};

// --- 3. RELAY CONTROLLER (UPDATED FOR SIM TOGGLE) ---
class RelayController {
    SerialPort* serial;
    bool lastState[9] = {false}; 
    bool firstRun = true;

    unsigned short crc16(const vector<unsigned char>& data) {
        unsigned short crc = 0xFFFF;
        for (size_t i = 0; i < data.size(); i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; } else { crc >>= 1; }
            }
        }
        return crc;
    }

public:
    RelayController(SerialPort* sp) : serial(sp) {}

    void setRelays(bool ev1, bool ev2, bool ev3, bool ev4, bool ev5, bool ev6, bool pump, bool heater) {
        // 1. Always update UI State (Visuals work in Sim mode)
        {
            lock_guard<mutex> lock(SYS.stateMutex);
            SYS.ev[1] = ev1; SYS.ev[2] = ev2; SYS.ev[3] = ev3;
            SYS.ev[4] = ev4; SYS.ev[5] = ev5; SYS.ev[6] = ev6;
            SYS.pump = pump; SYS.heater = heater;
        }

        // 2. Determine Hardware State
        // If Sim Mode is ON, force all hardware pins to FALSE (OFF)
        // If Live Mode is ON, use the actual logic values
        bool useSim = SYS.useRelaySimulation;
        
        bool hw_ev1 = useSim ? false : ev1;
        bool hw_ev2 = useSim ? false : ev2;
        bool hw_ev3 = useSim ? false : ev3;
        bool hw_ev4 = useSim ? false : ev4;
        bool hw_ev5 = useSim ? false : ev5;
        bool hw_ev6 = useSim ? false : ev6;
        bool hw_pump = useSim ? false : pump;
        bool hw_heat = useSim ? false : heater;

        bool desired[9] = {false, hw_ev1, hw_ev2, hw_ev3, hw_ev4, hw_ev5, hw_ev6, hw_pump, hw_heat};

        // 3. Send to Hardware (Smart Diff)
        for (int i = 1; i <= 8; i++) {
            if (firstRun || (desired[i] != lastState[i])) {
                unsigned char val = desired[i] ? 0x01 : 0x02; 
                vector<unsigned char> cmd = {0x01, 0x06, 0x00, (unsigned char)i, val, 0x00};
                unsigned short crc = crc16(cmd);
                cmd.push_back(crc & 0xFF);
                cmd.push_back((crc >> 8) & 0xFF);
                serial->write(cmd);
                Sleep(20); 
                lastState[i] = desired[i];
            }
        }
        firstRun = false;
    }
};

// --- 4. PHYSICS ENGINE ---
void CalculatePhysics() {
    string mode; { lock_guard<mutex> l(SYS.stateMutex); mode = SYS.currentMode; }
    
    if (mode == "WARM" || mode == "HOT") {
        double flow_kg_s = (SYS.massFlowRateL_min / 60.0);
        double target = (mode == "WARM") ? SYS.targetWarm : SYS.targetHot;
        double deltaT = target - SYS.t1; 
        if (deltaT < 0) deltaT = 0;

        double powerRequired = flow_kg_s * WATER_CP * deltaT;
        double duty = powerRequired / SYS.heaterMaxPowerW;
        if (duty > 1.0) duty = 1.0;
        
        SYS.calculatedPower = powerRequired;
        SYS.heaterDutyCycle = duty;
    } else {
        SYS.calculatedPower = 0;
        SYS.heaterDutyCycle = 0;
    }
}

// --- 5. MAIN LOGIC LOOP ---
void ControlLoop(RelayController* relays, SensorManager* sensors) {
    this_thread::sleep_for(chrono::seconds(2));

    while (true) {
        this_thread::sleep_for(chrono::milliseconds(100));
        
        // READ SENSORS
        sensors->read();

        // MAP VALUES (Sim vs Real)
        if (SYS.useSensorSimulation) {
            SYS.t1 = (double)SYS.simSensors[0];
            SYS.t2 = (double)SYS.simSensors[1];
            SYS.t3 = (double)SYS.simSensors[2];
            SYS.t4 = (double)SYS.simSensors[3];
        } else {
            SYS.t1 = (SYS.realSensors[0] < -900) ? 25.0 : (double)SYS.realSensors[0]; 
            SYS.t2 = (SYS.realSensors[1] < -900) ? 25.0 : (double)SYS.realSensors[1]; 
            SYS.t3 = (SYS.realSensors[2] < -900) ? 25.0 : (double)SYS.realSensors[2]; 
            SYS.t4 = (SYS.realSensors[3] < -900) ? 25.0 : (double)SYS.realSensors[3]; 
        }

        // READ STATE
        string mode;
        bool pouring;
        double t2, t3, tankTarget, tankDeltaLimit;
        {
            lock_guard<mutex> l(SYS.stateMutex);
            mode = SYS.currentMode;
            tankTarget = SYS.tankTargetTemp;
            tankDeltaLimit = SYS.tankMaxDelta;
        }
        t2 = SYS.t2; t3 = SYS.t3;
        pouring = SYS.isPouring;

        // HEATER PWM
        static auto pwmStart = chrono::steady_clock::now();
        long long ms = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - pwmStart).count();
        if (ms >= 1000) { pwmStart = chrono::steady_clock::now(); ms = 0; }
        bool heaterPwm = (ms < (SYS.heaterDutyCycle * 1000));

        bool r_ev[7] = {0}; bool r_pump = 0; bool r_heat = 0;

        // STATE MACHINE
        if (mode == "WARM" || mode == "HOT") {
            CalculatePhysics();
            static auto stateTimer = chrono::steady_clock::now();
            
            if (SYS.currentStage == "AUTO" || SYS.currentStage == "IDLE" || SYS.currentStage == "RECIRC") {
                SYS.currentStage = "PRE";
                stateTimer = chrono::steady_clock::now();
            }

            if (SYS.currentStage == "PRE") {
                r_heat = true; 
                SYS.statusMessage = (mode=="WARM"?"Warm":"Hot") + string(" Pre-heating...");
                if (chrono::steady_clock::now() - stateTimer > chrono::seconds(SYS.preTimeSec)) 
                    SYS.currentStage = "READY";
            }
            else if (SYS.currentStage == "READY") {
                SYS.statusMessage = "Ready. Hold Pour.";
                if (pouring) SYS.currentStage = "WHILE";
            }
            else if (SYS.currentStage == "WHILE") {
                if (!pouring) {
                    SYS.currentStage = "POST";
                    stateTimer = chrono::steady_clock::now();
                } else {
                    r_pump = true; r_heat = heaterPwm;
                    SYS.statusMessage = "Dispensing...";
                    if(mode=="WARM") { r_ev[2]=1; r_ev[6]=1; }
                    if(mode=="HOT")  { r_ev[1]=1; r_ev[4]=1; r_ev[6]=1; }
                }
            }
            else if (SYS.currentStage == "POST") {
                r_pump = true; r_ev[3]=1; r_ev[5]=1; 
                SYS.statusMessage = "Post-cycle...";
                if (chrono::steady_clock::now() - stateTimer > chrono::seconds(SYS.postTimeSec)) {
                    lock_guard<mutex> l(SYS.stateMutex);
                    SYS.currentMode = "AUTO"; 
                    SYS.currentStage = "IDLE";
                }
            }
        }
        else if (mode == "AUTO") {
            static auto recircTimer = chrono::steady_clock::now();
            bool needsHeat = (t3 < tankTarget);
            bool stratified = (abs(t3 - t2) > tankDeltaLimit);
            bool conditionsMet = (needsHeat || stratified);

            if (SYS.currentStage != "IDLE" && SYS.currentStage != "RECIRC" && SYS.currentStage != "POST") 
                SYS.currentStage = "IDLE";

            if (SYS.currentStage == "IDLE") {
                SYS.statusMessage = "System Idle (Temp OK)";
                if (conditionsMet) SYS.currentStage = "RECIRC";
            }
            else if (SYS.currentStage == "RECIRC") {
                if (!conditionsMet) {
                    SYS.currentStage = "POST";
                    recircTimer = chrono::steady_clock::now();
                } else {
                    r_ev[3] = 1; r_ev[5] = 1; r_pump = 1; r_heat = 1; 
                    SYS.statusMessage = "Recirculating...";
                }
            }
            else if (SYS.currentStage == "POST") {
                r_ev[3] = 1; r_ev[5] = 1; r_pump = 1; r_heat = 0; 
                SYS.statusMessage = "Recirc Done. Cooling...";
                if (chrono::steady_clock::now() - recircTimer > chrono::seconds(SYS.postTimeSec)) 
                    SYS.currentStage = "IDLE";
            }
        }

        relays->setRelays(r_ev[1], r_ev[2], r_ev[3], r_ev[4], r_ev[5], r_ev[6], r_pump, r_heat);
    }
}

// --- 6. WEB SERVER ---
void RunWebServer() {
    httplib::Server svr;

    svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
        string html = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<style>
    body { background: #121212; color: #e0e0e0; font-family: 'Segoe UI', sans-serif; padding: 20px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; max-width: 1200px; margin: auto; }
    .card { background: #1e1e1e; padding: 20px; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.5); }
    h2 { color: #bb86fc; border-bottom: 1px solid #333; padding-bottom:10px; margin-top:0; }
    
    .leds { display: grid; grid-template-columns: repeat(8, 1fr); text-align: center; font-size: 12px; }
    .led { width: 15px; height: 15px; border-radius: 50%; background: #333; margin: 5px auto; transition: 0.3s; box-shadow: inset 1px 1px 2px #000; }
    .led.on { background: #00ff00; box-shadow: 0 0 10px #00ff00; }
    .led.heat.on { background: #ff0000 !important; box-shadow: 0 0 15px #ff0000; }

    .btn-grp { display: flex; gap: 10px; margin: 15px 0; }
    button { flex: 1; padding: 15px; border: none; font-weight: bold; cursor: pointer; color: #000; border-radius: 4px; }
    .btn-warm { background: #ffb74d; } .btn-hot { background: #e57373; } .btn-auto { background: #64b5f6; }
    
    #pourBtn { width: 100%; font-size: 20px; background: #03dac6; margin-top: 15px; }
    #pourBtn:active { background: #018786; }

    /* SWITCH STYLES */
    .toggle-container { display: flex; align-items: center; justify-content: space-between; margin-bottom: 15px; background: #333; padding: 10px; border-radius: 5px; }
    .switch { position: relative; display: inline-block; width: 60px; height: 34px; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: .4s; border-radius: 34px; }
    .slider:before { position: absolute; content: ""; height: 26px; width: 26px; left: 4px; bottom: 4px; background-color: white; transition: .4s; border-radius: 50%; }
    input:checked + .slider { background-color: #2196F3; }
    input:checked + .slider:before { transform: translateX(26px); }

    .input-row { display: flex; justify-content: space-between; margin: 8px 0; align-items: center; }
    input[type=number] { width: 70px; background: #2c2c2c; border: 1px solid #444; color: white; padding: 5px; text-align:right;}
    input[type=range] { flex: 1; margin: 0 10px; }
    
    .live-data { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; font-family: monospace; color: #03dac6; font-size: 14px; }
    .live-box { background: #222; padding: 5px; text-align: center; border: 1px solid #444; border-radius: 4px; transition: 0.3s; }
    .live-box.disconnected { opacity: 0.3; border-color: #555; color: #555; }
</style>
</head>
<body>
<div class="grid">
    <div>
        <div class="card">
            <h2>Dispensing Control</h2>
            <div id="status" style="color: #bbb; height: 30px; font-style:italic;">Loading...</div>
            <div class="btn-grp">
                <button class="btn-warm" onclick="setMode('WARM')">WARM MODE</button>
                <button class="btn-hot" onclick="setMode('HOT')">HOT MODE</button>
                <button class="btn-auto" onclick="setMode('AUTO')">AUTO / CANCEL</button>
            </div>
            <button id="pourBtn" onmousedown="pour(1)" onmouseup="pour(0)" ontouchstart="pour(1)" ontouchend="pour(0)">HOLD TO POUR</button>
        </div>

        <div class="card" style="margin-top:20px">
            <h2>Input Source (Sensors)</h2>
            <div class="toggle-container">
                <span id="sourceLabel"><b>SIMULATION</b></span>
                <label class="switch">
                    <input type="checkbox" id="simToggle" checked onchange="toggleSim()">
                    <span class="slider"></span>
                </label>
            </div>
            
            <div id="simControls">
                <div class="input-row"><label>T1 Inlet</label><input type="range" min="10" max="40" value="20" oninput="setSens('t1',this.value)"><span id="v_t1">20</span></div>
                <div class="input-row"><label>T2 Tank Lo</label><input type="range" min="20" max="90" value="25" oninput="setSens('t2',this.value)"><span id="v_t2">25</span></div>
                <div class="input-row"><label>T3 Tank Hi</label><input type="range" min="20" max="90" value="55" oninput="setSens('t3',this.value)"><span id="v_t3">55</span></div>
            </div>
        </div>

        <div class="card" style="margin-top:20px">
            <h2>Live Readings</h2>
            <div class="live-data">
                <div class="live-box" id="box0">T0(Inlet)<br><span id="r0">--</span>°C</div>
                <div class="live-box" id="box1">T1(TkLo)<br><span id="r1">--</span>°C</div>
                <div class="live-box" id="box2">T2(TkHi)<br><span id="r2">--</span>°C</div>
                <div class="live-box" id="box3">T3(Out)<br><span id="r3">--</span>°C</div>
                <div class="live-box" id="box4">T4(Aux)<br><span id="r4">--</span>°C</div>
                <div class="live-box" id="box5">T5(Aux)<br><span id="r5">--</span>°C</div>
            </div>
        </div>
    </div>

    <div>
        <div class="card">
            <h2>Hardware Status</h2>
            <div class="toggle-container" style="border: 1px solid #444;">
                <span id="relayLabel">OUTPUT: <b>SIMULATED</b></span>
                <label class="switch">
                    <input type="checkbox" id="relayToggle" checked onchange="toggleRelay()">
                    <span class="slider"></span>
                </label>
            </div>
            <div class="leds">
                <div>E1<div id="l1" class="led"></div></div>
                <div>E2<div id="l2" class="led"></div></div>
                <div>E3<div id="l3" class="led"></div></div>
                <div>E4<div id="l4" class="led"></div></div>
                <div>E5<div id="l5" class="led"></div></div>
                <div>E6<div id="l6" class="led"></div></div>
                <div>PUMP<div id="lp" class="led"></div></div>
                <div>HEAT<div id="lh" class="led heat"></div></div>
            </div>
            <p>Stage: <b id="stage" style="color:#03dac6">IDLE</b></p>
        </div>

        <div class="card" style="margin-top:20px">
            <h2>Config & Physics</h2>
            <div class="input-row"><label>Mass Flow (L/min)</label><input type="number" id="c_flow" value="1.0" step="0.1" onchange="cfg()"></div>
            <div class="input-row"><label>Heater Power (W)</label><input type="number" id="c_pow" value="3000" step="100" onchange="cfg()"></div>
            <div class="input-row"><label>Power Calc:</label><span id="calc_w" style="color:#03dac6">0 W</span></div>
            <div class="input-row"><label>Duty Cycle:</label><span id="calc_d" style="color:#03dac6">0 %</span></div>
        </div>

        <div class="card" style="margin-top:20px">
            <h2>Logic Settings</h2>
            <div class="input-row"><label>Tank Target (°C)</label><input type="number" id="c_tt" value="65" onchange="cfg()"></div>
            <div class="input-row"><label>Max Delta T (°C)</label><input type="number" id="c_dt" value="2" onchange="cfg()"></div>
            <div class="input-row"><label>Target Warm (°C)</label><input type="number" id="c_tw" value="40" onchange="cfg()"></div>
            <div class="input-row"><label>Target Hot (°C)</label><input type="number" id="c_th" value="90" onchange="cfg()"></div>
        </div>
    </div>
</div>
<script>
    function setMode(m) { fetch('/cmd?mode='+m); }
    function pour(v) { fetch('/cmd?pour='+v); }
    function setSens(id, v) { document.getElementById('v_'+id).innerText=v; fetch('/sensor?id='+id+'&val='+v); }
    
    function toggleSim() { 
        let s = document.getElementById('simToggle').checked;
        document.getElementById('sourceLabel').innerHTML = "INPUT: <b>" + (s ? "SIMULATION" : "LIVE SENSORS") + "</b>";
        document.getElementById('simControls').style.opacity = s ? "1" : "0.3";
        fetch('/cmd?sim=' + (s?1:0));
    }

    function toggleRelay() {
        let s = document.getElementById('relayToggle').checked;
        document.getElementById('relayLabel').innerHTML = "OUTPUT: <b>" + (s ? "SIMULATED" : "LIVE RELAYS") + "</b>";
        // Visual indicator that relays are dangerous/live
        document.getElementById('relayLabel').style.color = s ? "#fff" : "#ff4444"; 
        fetch('/cmd?relaySim=' + (s?1:0));
    }
    
    function cfg() {
        let qs = `tt=${document.getElementById('c_tt').value}&dt=${document.getElementById('c_dt').value}` +
                 `&tw=${document.getElementById('c_tw').value}&th=${document.getElementById('c_th').value}` +
                 `&fl=${document.getElementById('c_flow').value}&pw=${document.getElementById('c_pow').value}`;
        fetch('/config?'+qs);
    }

    setInterval(()=>{
        fetch('/status').then(r=>r.json()).then(d=>{
            // LEDs
            for(let i=1;i<=6;i++) document.getElementById('l'+i).className='led '+(d.ev[i]?'on':'');
            document.getElementById('lp').className='led '+(d.pump?'on':'');
            document.getElementById('lh').className='led heat '+(d.heat?'on':'');
            
            // Text
            document.getElementById('status').innerText = d.msg;
            document.getElementById('stage').innerText = d.stage;
            document.getElementById('calc_w').innerText = d.power.toFixed(0) + " W";
            document.getElementById('calc_d').innerText = (d.duty*100).toFixed(0) + " %";
            
            // Real Sensors + Disconnect Logic
            for(let i=0; i<6; i++) {
                let val = d.real[i];
                let elBox = document.getElementById('box'+i);
                let elText = document.getElementById('r'+i);
                
                if(val <= -900) {
                    elText.innerText = "NC";
                    elBox.className = "live-box disconnected";
                } else {
                    elText.innerText = val.toFixed(1);
                    elBox.className = "live-box";
                }
            }
        });
    }, 500);
</script>
</body>
</html>
        )HTML";
        res.set_content(html, "text/html");
    });

    svr.Get("/status", [](const httplib::Request&, httplib::Response& res) {
        stringstream ss;
        ss << "{ \"ev\":[0," << SYS.ev[1] << "," << SYS.ev[2] << "," << SYS.ev[3] << "," 
           << SYS.ev[4] << "," << SYS.ev[5] << "," << SYS.ev[6] << "], \"pump\":" << SYS.pump 
           << ", \"heat\":" << SYS.heater << ", \"duty\":" << SYS.heaterDutyCycle
           << ", \"power\":" << SYS.calculatedPower
           << ", \"real\":[";
        for(int i=0;i<6;i++) ss << SYS.realSensors[i] << (i<5?",":"");
        ss << "]";
        { lock_guard<mutex> l(SYS.stateMutex); 
          ss << ", \"msg\":\"" << SYS.statusMessage << "\", \"stage\":\"" << SYS.currentStage << "\""; }
        ss << "}";
        res.set_content(ss.str(), "application/json");
    });

    svr.Get("/cmd", [](const httplib::Request& req, httplib::Response& res) {
        if(req.has_param("mode")) {
            lock_guard<mutex> l(SYS.stateMutex);
            SYS.currentMode = req.get_param_value("mode");
            SYS.currentStage = "AUTO"; 
        }
        if(req.has_param("pour")) SYS.isPouring = (req.get_param_value("pour") == "1");
        if(req.has_param("sim")) SYS.useSensorSimulation = (req.get_param_value("sim") == "1");
        if(req.has_param("relaySim")) SYS.useRelaySimulation = (req.get_param_value("relaySim") == "1");
        res.set_content("OK", "text/plain");
    });

    svr.Get("/sensor", [](const httplib::Request& req, httplib::Response& res) {
        string id = req.get_param_value("id"); double v = stod(req.get_param_value("val"));
        if(id=="t1") SYS.simSensors[0]=v; 
        if(id=="t2") SYS.simSensors[1]=v; 
        if(id=="t3") SYS.simSensors[2]=v;
        res.set_content("OK", "text/plain");
    });

    svr.Get("/config", [](const httplib::Request& req, httplib::Response& res) {
        try {
            if(req.has_param("tt")) SYS.tankTargetTemp = stod(req.get_param_value("tt"));
            if(req.has_param("dt")) SYS.tankMaxDelta = stod(req.get_param_value("dt"));
            if(req.has_param("tw")) SYS.targetWarm = stod(req.get_param_value("tw"));
            if(req.has_param("th")) SYS.targetHot = stod(req.get_param_value("th"));
            if(req.has_param("fl")) SYS.massFlowRateL_min = stod(req.get_param_value("fl"));
            if(req.has_param("pw")) SYS.heaterMaxPowerW = stod(req.get_param_value("pw"));
        } catch(...) {}
        res.set_content("OK", "text/plain");
    });

    cout << "System Running at http://localhost:8090" << endl;
    svr.listen("0.0.0.0", 8090);
}

// --- 7. MAIN ENTRY POINT ---
int main() {
    SerialPort p("COM9");
    RelayController r(&p);
    SensorManager s;
    
    // Initialize Sim Defaults
    SYS.simSensors[0] = 20.0; 
    SYS.simSensors[1] = 25.0; 
    SYS.simSensors[2] = 55.0; 
    SYS.simSensors[3] = 25.0;

    thread t(ControlLoop, &r, &s);
    t.detach();
    RunWebServer();
    return 0;
}