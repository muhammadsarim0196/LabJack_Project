/**
 * SMART HYDRAULIC CONTROL SYSTEM (FINAL)
 * --------------------------------------
 * - Logic: Auto-Recirc -> Post -> Idle
 * - Physics: m*Cp*dT Calculation Display & Config
 * - Hardware: DSD Tech / Modbus RTU Compatible
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

using namespace std;

// --- CONFIGURATION CONSTANTS ---
const double WATER_CP = 4186.0; // J/(kg*C)

// --- GLOBAL SHARED STATE ---
struct SystemState {
    // Simulated Sensors
    atomic<double> t1{20.0}; // Inlet
    atomic<double> t2{25.0}; // Tank Low
    atomic<double> t3{55.0}; // Tank High
    atomic<double> t4{25.0}; // Outlet

    // User Configuration
    atomic<double> targetWarm{40.0};
    atomic<double> targetHot{90.0};
    atomic<double> massFlowRateL_min{1.0};   // RESTORED
    atomic<double> heaterMaxPowerW{3000.0};  // RESTORED
    atomic<int>    preTimeSec{5};
    atomic<int>    postTimeSec{5};

    // Recirculation Config
    atomic<double> tankTargetTemp{65.0}; 
    atomic<double> tankMaxDelta{2.0};    

    // System Status
    atomic<double> calculatedPower{0.0};
    atomic<double> heaterDutyCycle{0.0};
    string currentMode = "AUTO";         
    string currentStage = "IDLE";        
    string statusMessage = "System Starting...";

    // Relay States
    bool ev[7] = {0}; 
    bool pump = false;
    bool heater = false;

    // Control Flags
    atomic<bool> isPouring{false}; 
    mutex stateMutex;
};

SystemState SYS;

// --- SERIAL PORT ---
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

// --- RELAY CONTROLLER (DSD TECH) ---
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

    void sendSingleRelay(int relayIndex, bool state) {
        // [ID] [06] [AddressHi] [AddressLo] [ValHi] [ValLo] [CRC]
        unsigned char val = state ? 0x01 : 0x02; 
        
        vector<unsigned char> cmd = {0x01, 0x06, 0x00, (unsigned char)relayIndex, val, 0x00};
        unsigned short crc = crc16(cmd);
        cmd.push_back(crc & 0xFF);
        cmd.push_back((crc >> 8) & 0xFF);
        serial->write(cmd);
        Sleep(20); 
    }

public:
    RelayController(SerialPort* sp) : serial(sp) {}

    void setRelays(bool ev1, bool ev2, bool ev3, bool ev4, bool ev5, bool ev6, bool pump, bool heater) {
        {
            lock_guard<mutex> lock(SYS.stateMutex);
            SYS.ev[1] = ev1; SYS.ev[2] = ev2; SYS.ev[3] = ev3;
            SYS.ev[4] = ev4; SYS.ev[5] = ev5; SYS.ev[6] = ev6;
            SYS.pump = pump; SYS.heater = heater;
        }

        bool desired[9] = {false, ev1, ev2, ev3, ev4, ev5, ev6, pump, heater};

        for (int i = 1; i <= 8; i++) {
            if (firstRun || (desired[i] != lastState[i])) {
                sendSingleRelay(i, desired[i]);
                lastState[i] = desired[i];
            }
        }
        firstRun = false;
    }
};

// --- PHYSICS ENGINE ---
void CalculatePhysics() {
    string mode; { lock_guard<mutex> l(SYS.stateMutex); mode = SYS.currentMode; }
    
    // Calculate for Dispensing
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
        // Clear physics values in Auto/Recirc mode (Heater is Bang-Bang there)
        SYS.calculatedPower = 0;
        SYS.heaterDutyCycle = 0;
    }
}

// --- MAIN LOGIC LOOP ---
void ControlLoop(RelayController* relays) {
    this_thread::sleep_for(chrono::seconds(2)); // Initial Start Gap

    while (true) {
        this_thread::sleep_for(chrono::milliseconds(100));

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

        // Software PWM
        static auto pwmStart = chrono::steady_clock::now();
        long long ms = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - pwmStart).count();
        if (ms >= 1000) { pwmStart = chrono::steady_clock::now(); ms = 0; }
        bool heaterPwm = (ms < (SYS.heaterDutyCycle * 1000));

        bool r_ev[7] = {0}; bool r_pump = 0; bool r_heat = 0;

        // --- DISPENSING (WARM/HOT) ---
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
                    SYS.currentMode = "AUTO"; // Back to Auto/Recirc
                    SYS.currentStage = "IDLE";
                }
            }
        }
        // --- AUTO RECIRCULATION ---
        else if (mode == "AUTO") {
            static auto recircTimer = chrono::steady_clock::now();
            
            bool needsHeat = (t3 < tankTarget);
            bool stratified = (abs(t3 - t2) > tankDeltaLimit);
            bool conditionsMet = (needsHeat || stratified);

            if (SYS.currentStage == "PRE" || SYS.currentStage == "READY" || SYS.currentStage == "WHILE") {
                SYS.currentStage = "IDLE"; // Reset from manual modes
            }

            if (SYS.currentStage == "IDLE") {
                r_ev[3]=0; r_ev[5]=0; r_pump=0; r_heat=0;
                SYS.statusMessage = "System Idle (Temp OK)";
                if (conditionsMet) {
                    SYS.currentStage = "RECIRC";
                }
            }
            else if (SYS.currentStage == "RECIRC") {
                // If conditions are NO LONGER met (Target reached) -> Go to POST
                if (!conditionsMet) {
                    SYS.currentStage = "POST";
                    recircTimer = chrono::steady_clock::now();
                } else {
                    r_ev[3] = 1; r_ev[5] = 1; r_pump = 1; r_heat = 1; // Full Heat
                    stringstream ss; ss << "Recirculating: ";
                    if(needsHeat) ss << "Heating Tank ";
                    if(stratified) ss << "Mixing Stratification";
                    SYS.statusMessage = ss.str();
                }
            }
            else if (SYS.currentStage == "POST") {
                // Recirc Post Routine
                r_ev[3] = 1; r_ev[5] = 1; r_pump = 1; r_heat = 0; // Pump ON, Heater OFF
                SYS.statusMessage = "Recirc Done. Post-Cooling...";
                
                if (chrono::steady_clock::now() - recircTimer > chrono::seconds(SYS.postTimeSec)) {
                    SYS.currentStage = "IDLE";
                }
            }
        }

        relays->setRelays(r_ev[1], r_ev[2], r_ev[3], r_ev[4], r_ev[5], r_ev[6], r_pump, r_heat);
    }
}

// --- WEB SERVER ---
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

    .input-row { display: flex; justify-content: space-between; margin: 8px 0; align-items: center; }
    input[type=number] { width: 70px; background: #2c2c2c; border: 1px solid #444; color: white; padding: 5px; text-align:right;}
    input[type=range] { flex: 1; margin: 0 10px; }
    .val-display { font-family: monospace; color: #03dac6; }
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
            <h2>Physics Parameters (Config)</h2>
            <div class="input-row"><label>Mass Flow (L/min)</label><input type="number" id="c_flow" value="1.0" step="0.1" onchange="cfg()"></div>
            <div class="input-row"><label>Heater Power (W)</label><input type="number" id="c_pow" value="3000" step="100" onchange="cfg()"></div>
            <hr style="border-color:#333">
            <div class="input-row"><label>Required Power:</label><span id="calc_w" class="val-display">0 W</span></div>
            <div class="input-row"><label>Duty Cycle:</label><span id="calc_d" class="val-display">0 %</span></div>
        </div>

        <div class="card" style="margin-top:20px">
            <h2>Test Simulation Sensors</h2>
            <div class="input-row"><label>T1 Inlet</label><input type="range" min="10" max="40" value="20" oninput="setSens('t1',this.value)"><span id="v_t1">20</span></div>
            <div class="input-row"><label>T2 Tank Lo</label><input type="range" min="20" max="90" value="25" oninput="setSens('t2',this.value)"><span id="v_t2">25</span></div>
            <div class="input-row"><label>T3 Tank Hi</label><input type="range" min="20" max="90" value="55" oninput="setSens('t3',this.value)"><span id="v_t3">55</span></div>
        </div>
    </div>

    <div>
        <div class="card">
            <h2>Hardware Status</h2>
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
            <h2>Auto-Recirc Settings</h2>
            <div class="input-row"><label>Tank Target (°C)</label><input type="number" id="c_tt" value="65" onchange="cfg()"></div>
            <div class="input-row"><label>Max Delta T (°C)</label><input type="number" id="c_dt" value="2" onchange="cfg()"></div>
        </div>
        
        <div class="card" style="margin-top:20px">
            <h2>Dispense Settings</h2>
            <div class="input-row"><label>Target Warm (°C)</label><input type="number" id="c_tw" value="40" onchange="cfg()"></div>
            <div class="input-row"><label>Target Hot (°C)</label><input type="number" id="c_th" value="90" onchange="cfg()"></div>
        </div>
    </div>
</div>
<script>
    function setMode(m) { fetch('/cmd?mode='+m); }
    function pour(v) { fetch('/cmd?pour='+v); }
    function setSens(id, v) { document.getElementById('v_'+id).innerText=v; fetch('/sensor?id='+id+'&val='+v); }
    function cfg() {
        let qs = `tt=${document.getElementById('c_tt').value}&dt=${document.getElementById('c_dt').value}` +
                 `&tw=${document.getElementById('c_tw').value}&th=${document.getElementById('c_th').value}` +
                 `&fl=${document.getElementById('c_flow').value}&pw=${document.getElementById('c_pow').value}`;
        fetch('/config?'+qs);
    }
    setInterval(()=>{
        fetch('/status').then(r=>r.json()).then(d=>{
            for(let i=1;i<=6;i++) document.getElementById('l'+i).className='led '+(d.ev[i]?'on':'');
            document.getElementById('lp').className='led '+(d.pump?'on':'');
            document.getElementById('lh').className='led heat '+(d.heat?'on':'');
            
            document.getElementById('status').innerText = d.msg;
            document.getElementById('stage').innerText = d.stage;
            document.getElementById('calc_w').innerText = d.power.toFixed(0) + " W";
            document.getElementById('calc_d').innerText = (d.duty*100).toFixed(0) + " %";
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
           << ", \"power\":" << SYS.calculatedPower;
        { lock_guard<mutex> l(SYS.stateMutex); 
          ss << ", \"msg\":\"" << SYS.statusMessage << "\", \"stage\":\"" << SYS.currentStage << "\""; }
        ss << "}";
        res.set_content(ss.str(), "application/json");
    });

    svr.Get("/cmd", [](const httplib::Request& req, httplib::Response& res) {
        if(req.has_param("mode")) {
            lock_guard<mutex> l(SYS.stateMutex);
            SYS.currentMode = req.get_param_value("mode");
            SYS.currentStage = "AUTO"; // Reset
        }
        if(req.has_param("pour")) SYS.isPouring = (req.get_param_value("pour") == "1");
        res.set_content("OK", "text/plain");
    });

    svr.Get("/sensor", [](const httplib::Request& req, httplib::Response& res) {
        string id = req.get_param_value("id"); double v = stod(req.get_param_value("val"));
        if(id=="t1") SYS.t1=v; if(id=="t2") SYS.t2=v; if(id=="t3") SYS.t3=v;
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

int main() {
    SerialPort p("COM9");
    RelayController r(&p);
    thread t(ControlLoop, &r);
    t.detach();
    RunWebServer();
    return 0;
}