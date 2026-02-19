/**
 * SMART HYDRAULIC CONTROL SYSTEM (.LOAD() COMPILER FIX)
 * -----------------------------------------------------
 * - Fixed std::atomic assignment deleted function errors
 * - Explicit .load() calls for thread-safe value extraction
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
const double MAX_POWER_PER_ZONE = 400.0; // 2x200W elements per zone
const long   HEATER_STAGGER_MS = 500;    // Delay between zone activations

// --- INFLUXDB CONFIGURATION ---
const char* INFLUX_HOST = "localhost";
const int   INFLUX_PORT = 8086;
const char* INFLUX_ORG  = "AQL";               
const char* INFLUX_BUCKET = "AQL_Bucket";      
const char* INFLUX_TOKEN = "6QfQ73FZc9Yw_BsfB1rX1dM2PQXivn_Hwd6jTMEBVunAlugvqo7u6Z5qjLa_rCokFdUOTuufOxQxeLVUhiAPeg=="; 

// --- GLOBAL SHARED STATE ---
struct SystemState {
    // Logic Inputs
    atomic<double> t1{20.0}; 
    atomic<double> t2{25.0}; 
    atomic<double> t3{55.0}; 
    atomic<double> t4{25.0}; 

    // Raw Real Sensor Data
    atomic<double> realSensors[6]; 

    // Toggles
    atomic<double> simSensors[4]; 
    atomic<bool>   useSensorSimulation{true}; 
    atomic<bool>   useRelaySimulation{true};  

    // User Config & Timers
    atomic<double> targetWarm{40.0};
    atomic<double> targetHot{90.0};
    atomic<double> massFlowRateL_min{1.0};
    atomic<double> heaterMaxLimitW{1000.0}; 
    atomic<int>    preTimeSec{5};           
    atomic<int>    postTimeSec{5};          
    atomic<double> tankTargetTemp{65.0}; 
    atomic<double> tankMaxDelta{2.0};    

    // Heat Flux Equalizer 
    atomic<double> eqInlet{10.0};
    atomic<double> eqCenter{10.0};
    atomic<double> eqOutlet{10.0};

    // System Status
    atomic<double> rawRequiredPower{0.0}; 
    atomic<bool>   powerExceeded{false};  
    atomic<double> totalCalculatedPower{0.0};
    atomic<double> zonePower[3]; 
    atomic<double> zoneDuty[3];  
    atomic<double> activeDuty[3]; 
    
    string currentMode = "AUTO";         
    string targetMode  = "AUTO";      
    string currentStage = "IDLE";        
    string statusMessage = "Initializing...";

    // Relays
    bool ev[7] = {0}; 
    bool pump = false;
    bool h[3] = {0}; 

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
                if(readValues[i] > -5.0 && readValues[i] < 110.0) {
                    SYS.realSensors[i] = readValues[i];
                } else {
                    SYS.realSensors[i] = -999.0; // NC Filter
                }
            }
        }
    }
    ~SensorManager() { if(connected) LJM_Close(handle); }
};

// --- 3. RELAY CONTROLLER ---
class RelayController {
    SerialPort* serial;
    bool lastState[12] = {false}; 
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

    void setRelays(bool ev1, bool ev2, bool ev3, bool ev4, bool ev5, bool ev6, bool pump, bool h1, bool h2, bool h3) {
        {
            lock_guard<mutex> lock(SYS.stateMutex);
            SYS.ev[1] = ev1; SYS.ev[2] = ev2; SYS.ev[3] = ev3;
            SYS.ev[4] = ev4; SYS.ev[5] = ev5; SYS.ev[6] = ev6;
            SYS.pump = pump; 
            SYS.h[0] = h1; SYS.h[1] = h2; SYS.h[2] = h3;
        }

        bool useSim = SYS.useRelaySimulation.load();
        bool desired[12] = {false};
        desired[1] = useSim ? false : ev1;
        desired[2] = useSim ? false : ev2;
        desired[3] = useSim ? false : ev3;
        desired[4] = useSim ? false : ev4;
        desired[5] = useSim ? false : ev5;
        desired[6] = useSim ? false : ev6;
        desired[7] = useSim ? false : pump;
        desired[9] = useSim ? false : h1; 
        desired[10]= useSim ? false : h2; 
        desired[11]= useSim ? false : h3; 

        for (int i = 1; i <= 11; i++) {
            if (i == 8) continue; 
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

// --- 4. INFLUXDB LOGGER ---
void InfluxLoggerThread() {
    httplib::Client cli(INFLUX_HOST, INFLUX_PORT);
    string tokenHeader = "Token " + string(INFLUX_TOKEN);
    cli.set_connection_timeout(1);

    while (true) {
        this_thread::sleep_for(chrono::seconds(1)); 
        
        // EXPLICIT LOAD() for all atomics
        double t1 = SYS.t1.load(), t2 = SYS.t2.load(), t3 = SYS.t3.load(), t4 = SYS.t4.load();
        double p1 = SYS.zonePower[0].load(), p2 = SYS.zonePower[1].load(), p3 = SYS.zonePower[2].load();
        
        string mode; 
        { lock_guard<mutex> l(SYS.stateMutex); mode = SYS.currentMode; }

        stringstream ss; ss << fixed << setprecision(2);
        ss << "thermal_system,unit=labjack,mode=" << mode 
           << " t1=" << t1 << ",t2=" << t2 << ",t3=" << t3 << ",t4=" << t4
           << ",power_inlet=" << p1 << ",power_center=" << p2 << ",power_outlet=" << p3
           << ",pump=" << (SYS.pump ? 1 : 0);

        string path = "/api/v2/write?org=" + string(INFLUX_ORG) + "&bucket=" + string(INFLUX_BUCKET) + "&precision=s";
        httplib::Headers headers = { {"Authorization", tokenHeader}, {"Content-Type", "text/plain"} };
        cli.Post(path.c_str(), headers, ss.str(), "text/plain");
    }
}

// --- 5. PHYSICS ENGINE ---
void CalculatePhysics() {
    string mode; { lock_guard<mutex> l(SYS.stateMutex); mode = SYS.currentMode; }
    
    if (mode == "WARM" || mode == "HOT") {
        // EXPLICIT LOAD() for all atomics
        double flow_kg_s = (SYS.massFlowRateL_min.load() / 60.0);
        double deltaT = 0.0;

        if (mode == "WARM") deltaT = SYS.targetWarm.load() - SYS.t1.load();
        else if (mode == "HOT") deltaT = SYS.targetHot.load() - SYS.t3.load(); 

        if (deltaT < 0) deltaT = 0; 

        double powerRequired = flow_kg_s * WATER_CP * deltaT;
        SYS.rawRequiredPower = powerRequired;
        double globalLimit = SYS.heaterMaxLimitW.load();
        SYS.powerExceeded = (powerRequired > globalLimit); 
        double activePower = min(powerRequired, globalLimit);

        double r1 = SYS.eqInlet.load(), r2 = SYS.eqCenter.load(), r3 = SYS.eqOutlet.load();
        double totalRatio = r1 + r2 + r3;
        if (totalRatio < 0.01) totalRatio = 0.01; 

        double p1 = activePower * (r1 / totalRatio);
        double p2 = activePower * (r2 / totalRatio);
        double p3 = activePower * (r3 / totalRatio);

        double d1 = p1 / MAX_POWER_PER_ZONE; if(d1 > 1.0) d1 = 1.0;
        double d2 = p2 / MAX_POWER_PER_ZONE; if(d2 > 1.0) d2 = 1.0;
        double d3 = p3 / MAX_POWER_PER_ZONE; if(d3 > 1.0) d3 = 1.0;

        SYS.zoneDuty[0] = d1; SYS.zonePower[0] = d1 * MAX_POWER_PER_ZONE;
        SYS.zoneDuty[1] = d2; SYS.zonePower[1] = d2 * MAX_POWER_PER_ZONE;
        SYS.zoneDuty[2] = d3; SYS.zonePower[2] = d3 * MAX_POWER_PER_ZONE;
        SYS.totalCalculatedPower = SYS.zonePower[0].load() + SYS.zonePower[1].load() + SYS.zonePower[2].load();
        
    } else {
        SYS.totalCalculatedPower = 0;
        SYS.rawRequiredPower = 0;
        SYS.powerExceeded = false;
        for(int i=0; i<3; i++) { SYS.zonePower[i] = MAX_POWER_PER_ZONE; SYS.zoneDuty[i] = 1.0; }
    }
}

// --- 6. MAIN LOGIC LOOP ---
void ControlLoop(RelayController* relays, SensorManager* sensors) {
    this_thread::sleep_for(chrono::seconds(2));

    auto stateTimer = chrono::steady_clock::now();

    while (true) {
        this_thread::sleep_for(chrono::milliseconds(100));
        sensors->read();

        // EXPLICIT LOAD() for all mappings
        if (SYS.useSensorSimulation.load()) {
            SYS.t1 = SYS.simSensors[0].load(); 
            SYS.t2 = SYS.simSensors[1].load();
            SYS.t3 = SYS.simSensors[2].load(); 
            SYS.t4 = SYS.simSensors[3].load();
        } else {
            SYS.t1 = (SYS.realSensors[0].load() < -900) ? 25.0 : SYS.realSensors[0].load(); 
            SYS.t2 = (SYS.realSensors[1].load() < -900) ? 25.0 : SYS.realSensors[1].load(); 
            SYS.t3 = (SYS.realSensors[2].load() < -900) ? 25.0 : SYS.realSensors[2].load(); 
            SYS.t4 = (SYS.realSensors[3].load() < -900) ? 25.0 : SYS.realSensors[3].load(); 
        }

        string mode, targetMode;
        double t2, t3, tankTarget, tankDeltaLimit;
        bool pouring;
        {
            lock_guard<mutex> l(SYS.stateMutex);
            mode = SYS.currentMode;
            targetMode = SYS.targetMode;
        }
        
        // EXPLICIT LOAD() to avoid compiler confusion
        t2 = SYS.t2.load(); 
        t3 = SYS.t3.load();
        tankTarget = SYS.tankTargetTemp.load();
        tankDeltaLimit = SYS.tankMaxDelta.load();
        pouring = SYS.isPouring.load();

        // --- SAFE TRANSITION LOGIC ---
        if (mode != targetMode) {
            if (SYS.currentStage == "IDLE" || SYS.currentStage == "READY") {
                lock_guard<mutex> l(SYS.stateMutex);
                SYS.currentMode = targetMode;
                SYS.currentStage = (targetMode == "AUTO") ? "IDLE" : "READY";
                stateTimer = chrono::steady_clock::now(); 
                mode = targetMode;
            } else if (SYS.currentStage != "POST") {
                SYS.currentStage = "POST";
                stateTimer = chrono::steady_clock::now();
            } else {
                SYS.statusMessage = "Cooling down before switching to " + targetMode + "...";
            }
        }

        CalculatePhysics();
        string warningText = SYS.powerExceeded.load() ? " | LIMIT EXCEEDED" : "";

        // --- STATE MACHINE ---
        bool r_ev[7] = {0}; bool r_pump = 0; 
        bool stageWantsHeat = false;

        if (mode == "WARM" || mode == "HOT") {
            if (SYS.currentStage == "READY") {
                auto elapsed = chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - stateTimer).count();
                if (elapsed >= 20) { 
                    lock_guard<mutex> l(SYS.stateMutex);
                    SYS.targetMode = "AUTO"; 
                } else {
                    SYS.statusMessage = string(mode == "WARM" ? "Warm" : "Hot") + " Mode. Ready (" + to_string(20 - elapsed) + "s) " + warningText;
                    if (pouring) {
                        SYS.currentStage = "PRE";
                        stateTimer = chrono::steady_clock::now();
                    }
                }
            }
            else if (SYS.currentStage == "PRE") {
                if (!pouring) {
                    SYS.currentStage = "POST";
                    stateTimer = chrono::steady_clock::now();
                } else {
                    stageWantsHeat = true;
                    auto elapsed = chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - stateTimer).count();
                    long rem = SYS.preTimeSec.load() - elapsed;
                    if (rem <= 0) SYS.currentStage = "WHILE"; 
                    else SYS.statusMessage = "Pre-heating... " + to_string(rem) + "s" + warningText;
                }
            }
            else if (SYS.currentStage == "WHILE") {
                if (!pouring) {
                    SYS.currentStage = "POST";
                    stateTimer = chrono::steady_clock::now();
                } else {
                    r_pump = true; stageWantsHeat = true;
                    SYS.statusMessage = "Dispensing..." + warningText;
                    if(mode=="WARM") { r_ev[2]=1; r_ev[6]=1; }
                    if(mode=="HOT")  { r_ev[1]=1; r_ev[4]=1; r_ev[6]=1; }
                }
            }
            else if (SYS.currentStage == "POST") {
                r_pump = true; r_ev[3]=1; r_ev[5]=1; 
                auto elapsed = chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - stateTimer).count();
                long rem = SYS.postTimeSec.load() - elapsed;
                if (rem <= 0) {
                    SYS.currentStage = "READY"; 
                    stateTimer = chrono::steady_clock::now(); 
                } else {
                    SYS.statusMessage = "Post-cycle cooling... " + to_string(rem) + "s";
                }
            }
        }
        else if (mode == "AUTO") {
            bool needsHeat = (t3 < tankTarget);
            bool stratified = (abs(t3 - t2) > tankDeltaLimit);
            bool conditionsMet = (needsHeat || stratified);

            if (SYS.currentStage == "IDLE") {
                SYS.statusMessage = "System Idle (Temp OK)";
                if (conditionsMet) SYS.currentStage = "RECIRC";
            }
            else if (SYS.currentStage == "RECIRC") {
                if (!conditionsMet) {
                    SYS.currentStage = "POST";
                    stateTimer = chrono::steady_clock::now();
                } else {
                    r_ev[3] = 1; r_ev[5] = 1; r_pump = 1; stageWantsHeat = true;
                    SYS.statusMessage = "Recirculating...";
                }
            }
            else if (SYS.currentStage == "POST") {
                r_ev[3] = 1; r_ev[5] = 1; r_pump = 1; 
                auto elapsed = chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - stateTimer).count();
                long rem = SYS.postTimeSec.load() - elapsed;
                if (rem <= 0) SYS.currentStage = "IDLE";
                else SYS.statusMessage = "Recirc Done. Cooling... " + to_string(rem) + "s";
            }
        }

        // --- STAGGERED HEATER LOGIC ---
        static bool lastHeatingEnabled = false;
        static auto heatChangeTime = chrono::steady_clock::now();

        if (stageWantsHeat != lastHeatingEnabled) {
            lastHeatingEnabled = stageWantsHeat;
            heatChangeTime = chrono::steady_clock::now();
        }

        long heatElapsedMs = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - heatChangeTime).count();
        bool allowH[3] = {false, false, false}; 

        if (stageWantsHeat) {
            allowH[0] = true;
            allowH[1] = (heatElapsedMs > HEATER_STAGGER_MS);
            allowH[2] = (heatElapsedMs > (HEATER_STAGGER_MS * 2));
        } else {
            allowH[2] = false;
            allowH[1] = (heatElapsedMs < HEATER_STAGGER_MS);
            allowH[0] = (heatElapsedMs < (HEATER_STAGGER_MS * 2));
        }

        static auto pwmStart = chrono::steady_clock::now();
        long long ms = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - pwmStart).count();
        if (ms >= 1000) { pwmStart = chrono::steady_clock::now(); ms = 0; }
        
        bool r_h[3];
        for (int i=0; i<3; i++) {
            r_h[i] = allowH[i] && (ms < (SYS.zoneDuty[i].load() * 1000));
            SYS.activeDuty[i] = allowH[i] ? SYS.zoneDuty[i].load() : 0.0; 
        }

        relays->setRelays(r_ev[1], r_ev[2], r_ev[3], r_ev[4], r_ev[5], r_ev[6], r_pump, r_h[0], r_h[1], r_h[2]);
    }
}

// --- 7. WEB SERVER ---
void RunWebServer() {
    httplib::Server svr;

    svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
        string html = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<style>
    body { background: #121212; color: #e0e0e0; font-family: 'Segoe UI', sans-serif; padding: 20px; font-size: 14px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; max-width: 1300px; margin: auto; }
    .card { background: #1e1e1e; padding: 15px; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.5); margin-bottom: 20px;}
    h2 { color: #bb86fc; border-bottom: 1px solid #333; padding-bottom:5px; margin-top:0; font-size: 18px;}
    
    .leds { display: grid; grid-template-columns: repeat(10, 1fr); text-align: center; font-size: 11px; }
    .led { width: 14px; height: 14px; border-radius: 50%; background: #333; margin: 5px auto; transition: 0.3s; box-shadow: inset 1px 1px 2px #000; }
    .led.on { background: #00ff00; box-shadow: 0 0 10px #00ff00; }
    .led.heat.on { background: #ff0000 !important; box-shadow: 0 0 15px #ff0000; }

    .btn-grp { display: flex; gap: 10px; margin: 10px 0; }
    button { flex: 1; padding: 12px; border: none; font-weight: bold; cursor: pointer; color: #000; border-radius: 4px; }
    .btn-warm { background: #ffb74d; } .btn-hot { background: #e57373; } .btn-auto { background: #64b5f6; }
    #pourBtn { width: 100%; font-size: 18px; background: #03dac6; margin-top: 10px; }
    #pourBtn:active { background: #018786; }

    .toggle-container { display: flex; align-items: center; justify-content: space-between; margin-bottom: 10px; background: #333; padding: 8px; border-radius: 5px; }
    .switch { position: relative; display: inline-block; width: 50px; height: 26px; }
    .switch input { opacity: 0; width: 0; height: 0; }
    .slider { position: absolute; cursor: pointer; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; transition: .4s; border-radius: 34px; }
    .slider:before { position: absolute; content: ""; height: 18px; width: 18px; left: 4px; bottom: 4px; background-color: white; transition: .4s; border-radius: 50%; }
    input:checked + .slider { background-color: #2196F3; }
    input:checked + .slider:before { transform: translateX(24px); }

    .input-row { display: flex; justify-content: space-between; margin: 6px 0; align-items: center; }
    input[type=number] { width: 60px; background: #2c2c2c; border: 1px solid #444; color: white; padding: 4px; text-align:right;}
    input[type=range] { flex: 1; margin: 0 10px; }
    
    .live-data { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; font-family: monospace; color: #03dac6; font-size: 12px; }
    .live-box { background: #222; padding: 5px; text-align: center; border: 1px solid #444; border-radius: 4px; }
    .live-box.disconnected { opacity: 0.3; border-color: #555; color: #555; }

    .eq-grid { display: grid; grid-template-columns: 1fr 2fr 1fr; gap: 5px; text-align: center; font-family: monospace; font-size: 12px; margin-bottom: 10px;}
    .val-display { color: #ffb74d; }
    .warning { color: #ff4444; font-weight: bold; }
</style>
</head>
<body>
<div class="grid">
    <div>
        <div class="card">
            <h2>Dispensing Control</h2>
            <div id="status" style="color: #bbb; height: 40px; font-style:italic; font-size:16px;">Loading...</div>
            <div class="btn-grp">
                <button class="btn-warm" onclick="setMode('WARM')">WARM MODE</button>
                <button class="btn-hot" onclick="setMode('HOT')">HOT MODE</button>
                <button class="btn-auto" onclick="setMode('AUTO')">AUTO / CANCEL</button>
            </div>
            <button id="pourBtn" onmousedown="pour(1)" onmouseup="pour(0)" ontouchstart="pour(1)" ontouchend="pour(0)">HOLD TO POUR</button>
        </div>

        <div class="card">
            <h2>Hardware Sensors (Live)</h2>
            <div class="live-data">
                <div class="live-box" id="box0">T0(Inlet)<br><span id="r0">--</span>°C</div>
                <div class="live-box" id="box1">T1(TkLo)<br><span id="r1">--</span>°C</div>
                <div class="live-box" id="box2">T2(TkHi)<br><span id="r2">--</span>°C</div>
                <div class="live-box" id="box3">T3(Out)<br><span id="r3">--</span>°C</div>
                <div class="live-box" id="box4">T4(Aux)<br><span id="r4">--</span>°C</div>
                <div class="live-box" id="box5">T5(Aux)<br><span id="r5">--</span>°C</div>
            </div>
            <br>
            <div class="toggle-container">
                <span id="sourceLabel">INPUT: <b>SIMULATION</b></span>
                <label class="switch"><input type="checkbox" id="simToggle" checked onchange="toggleSim()"><span class="slider"></span></label>
            </div>
            <div id="simControls">
                <div class="input-row"><label>T1 Inlet</label><input type="range" min="10" max="40" value="20" oninput="setSens('t1',this.value)"><span id="v_t1">20</span></div>
                <div class="input-row"><label>T2 Tank Lo</label><input type="range" min="20" max="90" value="25" oninput="setSens('t2',this.value)"><span id="v_t2">25</span></div>
                <div class="input-row"><label>T3 Tank Hi</label><input type="range" min="20" max="90" value="55" oninput="setSens('t3',this.value)"><span id="v_t3">55</span></div>
            </div>
        </div>
    </div>

    <div>
        <div class="card">
            <h2>Hardware Status</h2>
            <div class="toggle-container" style="border: 1px solid #444;">
                <span id="relayLabel">OUTPUT: <b>SIMULATED</b></span>
                <label class="switch"><input type="checkbox" id="relayToggle" checked onchange="toggleRelay()"><span class="slider"></span></label>
            </div>
            <div class="leds">
                <div>E1<div id="l1" class="led"></div></div>
                <div>E2<div id="l2" class="led"></div></div>
                <div>E3<div id="l3" class="led"></div></div>
                <div>E4<div id="l4" class="led"></div></div>
                <div>E5<div id="l5" class="led"></div></div>
                <div>E6<div id="l6" class="led"></div></div>
                <div>PMP<div id="lp" class="led"></div></div>
                <div>H1<div id="h0" class="led heat"></div></div>
                <div>H2<div id="h1" class="led heat"></div></div>
                <div>H3<div id="h2" class="led heat"></div></div>
            </div>
            <p style="margin: 10px 0 0 0;">Stage: <b id="stage" style="color:#03dac6">IDLE</b></p>
        </div>

        <div class="card">
            <h2>Heat Flux Equalizer</h2>
            <div class="input-row"><label>Mass Flow (L/min)</label><input type="number" id="c_flow" value="1.0" step="0.1" onchange="cfg()"></div>
            <div class="input-row"><label>Global Limit (W)</label><input type="number" id="c_pow" value="1000" step="100" onchange="cfg()"></div>
            <div class="input-row"><label>Power Calc:</label><span id="calc_w" style="color:#03dac6">0 W</span></div>
            
            <div class="eq-grid" style="color:#bbb; border-bottom:1px solid #333; padding-bottom:5px; margin-top:10px;">
                <div>ZONE</div><div>FLUX BIAS</div><div>POWER / DUTY</div>
            </div>
            <div class="eq-grid">
                <div>INLET (H1)</div>
                <div><input type="range" min="0" max="10" value="10" id="eq_1" onchange="cfg()" oninput="document.getElementById('ev1').innerText=this.value"> <span id="ev1">10</span></div>
                <div><span id="z1_p" class="val-display">0</span>W / <span id="z1_d" class="val-display">0</span>%</div>
            </div>
            <div class="eq-grid">
                <div>CENTER (H2)</div>
                <div><input type="range" min="0" max="10" value="10" id="eq_2" onchange="cfg()" oninput="document.getElementById('ev2').innerText=this.value"> <span id="ev2">10</span></div>
                <div><span id="z2_p" class="val-display">0</span>W / <span id="z2_d" class="val-display">0</span>%</div>
            </div>
            <div class="eq-grid">
                <div>OUTLET (H3)</div>
                <div><input type="range" min="0" max="10" value="10" id="eq_3" onchange="cfg()" oninput="document.getElementById('ev3').innerText=this.value"> <span id="ev3">10</span></div>
                <div><span id="z3_p" class="val-display">0</span>W / <span id="z3_d" class="val-display">0</span>%</div>
            </div>
        </div>

        <div class="card">
            <h2>Logic & Timers</h2>
            <div class="input-row" style="width:48%; display:inline-flex;"><label>Tk Target</label><input type="number" id="c_tt" value="65" onchange="cfg()"></div>
            <div class="input-row" style="width:48%; display:inline-flex; float:right;"><label>Max dTk</label><input type="number" id="c_dt" value="2" onchange="cfg()"></div>
            <div class="input-row" style="width:48%; display:inline-flex;"><label>Warm °C</label><input type="number" id="c_tw" value="40" onchange="cfg()"></div>
            <div class="input-row" style="width:48%; display:inline-flex; float:right;"><label>Hot °C</label><input type="number" id="c_th" value="90" onchange="cfg()"></div>
            <div class="input-row" style="width:48%; display:inline-flex;"><label>Pre (s)</label><input type="number" id="c_pre" value="5" onchange="cfg()"></div>
            <div class="input-row" style="width:48%; display:inline-flex; float:right;"><label>Post (s)</label><input type="number" id="c_pos" value="5" onchange="cfg()"></div>
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
        document.getElementById('relayLabel').style.color = s ? "#fff" : "#ff4444"; 
        fetch('/cmd?relaySim=' + (s?1:0));
    }
    function cfg() {
        let qs = `tt=${document.getElementById('c_tt').value}&dt=${document.getElementById('c_dt').value}` +
                 `&tw=${document.getElementById('c_tw').value}&th=${document.getElementById('c_th').value}` +
                 `&fl=${document.getElementById('c_flow').value}&pw=${document.getElementById('c_pow').value}` +
                 `&pre=${document.getElementById('c_pre').value}&pos=${document.getElementById('c_pos').value}` +
                 `&e1=${document.getElementById('eq_1').value}&e2=${document.getElementById('eq_2').value}&e3=${document.getElementById('eq_3').value}`;
        fetch('/config?'+qs);
    }

    setInterval(()=>{
        fetch('/status').then(r=>r.json()).then(d=>{
            // Relays
            for(let i=1;i<=6;i++) document.getElementById('l'+i).className='led '+(d.ev[i]?'on':'');
            document.getElementById('lp').className='led '+(d.pump?'on':'');
            for(let i=0;i<3;i++) document.getElementById('h'+i).className='led heat '+(d.h[i]?'on':'');
            
            // Text & Status 
            document.getElementById('status').innerHTML = d.msg.replace('|', '<br>');
            document.getElementById('stage').innerText = d.stage;
            
            // Physics limits warning
            if (d.pEx) {
                document.getElementById('calc_w').innerHTML = "<span class='warning'>" + d.rawW.toFixed(0) + "W (Need)</span> / " + d.totW.toFixed(0) + "W (Limit)";
            } else {
                document.getElementById('calc_w').innerHTML = "<span style='color:#03dac6'>" + d.totW.toFixed(0) + "W</span>";
            }

            for(let i=0;i<3;i++) {
                document.getElementById('z'+(i+1)+'_p').innerText = d.zW[i].toFixed(0);
                document.getElementById('z'+(i+1)+'_d').innerText = (d.zD[i]*100).toFixed(0);
            }
            
            // Live Sensors
            for(let i=0; i<6; i++) {
                let val = d.real[i];
                let elBox = document.getElementById('box'+i);
                let elText = document.getElementById('r'+i);
                if(val <= -900) { elText.innerText = "NC"; elBox.className = "live-box disconnected"; } 
                else { elText.innerText = val.toFixed(1); elBox.className = "live-box"; }
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
        // EXPLICIT LOAD() for JSON output
        ss << "{ \"ev\":[0," << SYS.ev[1] << "," << SYS.ev[2] << "," << SYS.ev[3] << "," 
           << SYS.ev[4] << "," << SYS.ev[5] << "," << SYS.ev[6] << "], \"pump\":" << SYS.pump 
           << ", \"h\":[" << SYS.h[0] << "," << SYS.h[1] << "," << SYS.h[2] << "]"
           << ", \"totW\":" << SYS.totalCalculatedPower.load()
           << ", \"rawW\":" << SYS.rawRequiredPower.load()
           << ", \"pEx\":" << (SYS.powerExceeded.load() ? "true" : "false")
           << ", \"zW\":[" << SYS.zonePower[0].load() << "," << SYS.zonePower[1].load() << "," << SYS.zonePower[2].load() << "]"
           << ", \"zD\":[" << SYS.activeDuty[0].load() << "," << SYS.activeDuty[1].load() << "," << SYS.activeDuty[2].load() << "]"
           << ", \"real\":[";
        for(int i=0;i<6;i++) ss << SYS.realSensors[i].load() << (i<5?",":"");
        ss << "]";
        { lock_guard<mutex> l(SYS.stateMutex); 
          ss << ", \"msg\":\"" << SYS.statusMessage << "\", \"stage\":\"" << SYS.currentStage << "\""; }
        ss << "}";
        res.set_content(ss.str(), "application/json");
    });

    svr.Get("/cmd", [](const httplib::Request& req, httplib::Response& res) {
        if(req.has_param("mode")) {
            lock_guard<mutex> l(SYS.stateMutex);
            SYS.targetMode = req.get_param_value("mode");
        }
        if(req.has_param("pour")) SYS.isPouring = (req.get_param_value("pour") == "1");
        if(req.has_param("sim")) SYS.useSensorSimulation = (req.get_param_value("sim") == "1");
        if(req.has_param("relaySim")) SYS.useRelaySimulation = (req.get_param_value("relaySim") == "1");
        res.set_content("OK", "text/plain");
    });

    svr.Get("/sensor", [](const httplib::Request& req, httplib::Response& res) {
        string id = req.get_param_value("id"); double v = stod(req.get_param_value("val"));
        if(id=="t1") SYS.simSensors[0] = v; 
        if(id=="t2") SYS.simSensors[1] = v; 
        if(id=="t3") SYS.simSensors[2] = v;
        res.set_content("OK", "text/plain");
    });

    svr.Get("/config", [](const httplib::Request& req, httplib::Response& res) {
        try {
            if(req.has_param("tt")) SYS.tankTargetTemp = stod(req.get_param_value("tt"));
            if(req.has_param("dt")) SYS.tankMaxDelta = stod(req.get_param_value("dt"));
            if(req.has_param("tw")) SYS.targetWarm = stod(req.get_param_value("tw"));
            if(req.has_param("th")) SYS.targetHot = stod(req.get_param_value("th"));
            if(req.has_param("fl")) SYS.massFlowRateL_min = stod(req.get_param_value("fl"));
            if(req.has_param("pw")) SYS.heaterMaxLimitW = stod(req.get_param_value("pw"));
            if(req.has_param("pre")) SYS.preTimeSec = stoi(req.get_param_value("pre"));
            if(req.has_param("pos")) SYS.postTimeSec = stoi(req.get_param_value("pos"));
            if(req.has_param("e1")) SYS.eqInlet = stod(req.get_param_value("e1"));
            if(req.has_param("e2")) SYS.eqCenter = stod(req.get_param_value("e2"));
            if(req.has_param("e3")) SYS.eqOutlet = stod(req.get_param_value("e3"));
        } catch(...) {}
        res.set_content("OK", "text/plain");
    });

    cout << "System Running at http://localhost:8090" << endl;
    svr.listen("0.0.0.0", 8090);
}

// --- 8. MAIN ENTRY POINT ---
int main() {
    SerialPort p("COM9");
    RelayController r(&p);
    SensorManager s;
    
    SYS.simSensors[0] = 20.0; SYS.simSensors[1] = 25.0; 
    SYS.simSensors[2] = 55.0; SYS.simSensors[3] = 25.0;

    thread logger(InfluxLoggerThread); logger.detach();
    thread t(ControlLoop, &r, &s); t.detach();
    
    RunWebServer();
    return 0;
}