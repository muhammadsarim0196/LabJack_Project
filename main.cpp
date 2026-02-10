/**
 * LABJACK CONTROL SYSTEM + WEB CONTROLLER (ROBUST STRING FIX)
 * ---------------------------------------
 * Hardware: LabJack T7 (Mux80) + DSD Tech RS485 Relay
 * Software: InfluxDB Logger + Web Control Panel
 */

// 1. NETWORK HEADERS MUST BE FIRST
#define WIN32_LEAN_AND_MEAN 
#include <winsock2.h> 
#include <windows.h>

#include <iostream>
#include <vector>
#include <string>
#include <atomic> 
#include <LabJackM.h>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include "httplib.h"

using namespace std;

// --- SHARED VARIABLES ---
atomic<double> GLOBAL_TARGET_TEMP(30.0); 

// --- CONFIGURATION ---
const char* INFLUX_HOST = "localhost";
const int   INFLUX_PORT = 8086;
const char* INFLUX_ORG  = "AQL";               
const char* INFLUX_BUCKET = "AQL_Bucket";      
const char* INFLUX_TOKEN = "6QfQ73FZc9Yw_BsfB1rX1dM2PQXivn_Hwd6jTMEBVunAlugvqo7u6Z5qjLa_rCokFdUOTuufOxQxeLVUhiAPeg=="; 

// --- HELPER FUNCTION ---
void ErrorCheck(int err, const char *msg) { 
    if(err < 0) { 
        char errName[LJM_MAX_NAME_SIZE];
        LJM_ErrorToString(err, errName);
        printf("Error: %s (%s)\n", msg, errName); 
    } 
}

// --- WEB SERVER THREAD ---
void RunWebServer() {
    httplib::Server svr;

    // 1. HOST THE WEBPAGE
    // FIX: Using R"HTML( ... )HTML" to prevent parsing errors
    svr.Get("/", [](const httplib::Request&, httplib::Response& res) {
        string html = R"HTML(
            <html>
            <head>
                <style>
                    body { font-family: sans-serif; text-align: center; padding: 50px; background-color: #222; color: white; }
                    input { padding: 10px; font-size: 20px; width: 100px; text-align: center;}
                    button { padding: 10px 20px; font-size: 20px; cursor: pointer; background-color: #00CC00; border: none; font-weight: bold;}
                    h1 { color: #10ff10; }
                </style>
            </head>
            <body>
                <h1>Heater Control Panel</h1>
                <p>Current Target Temperature:</p>
                <h2 id="disp">Loading...</h2>
                <br>
                <input type="number" id="newTemp" placeholder="30">
                <button onclick="setTemp()">Set Temp</button>

                <script>
                    function setTemp() {
                        var val = document.getElementById('newTemp').value;
                        fetch("/set?val=" + val).then(() => {
                            alert("Target updated to " + val + "C");
                            updateDisplay();
                        });
                    }
                    
                    function updateDisplay() {
                        fetch("/status").then(r => r.text()).then(t => {
                            document.getElementById("disp").innerText = t + " C";
                        });
                    }
                    setInterval(updateDisplay, 1000); 
                </script>
            </body>
            </html>
        )HTML"; // <--- CLOSING DELIMITER MUST MATCH OPENING
        
        res.set_content(html, "text/html");
    });

    // 2. RECEIVE COMMANDS
    svr.Get("/set", [](const httplib::Request& req, httplib::Response& res) {
        if (req.has_param("val")) {
            string val = req.get_param_value("val");
            try {
                GLOBAL_TARGET_TEMP = stod(val); 
                cout << "\n[WEB CMD] New Target Set: " << val << " C" << endl;
            } catch(...) {
                cout << "\n[WEB CMD] Invalid Number Received" << endl;
            }
        }
        res.set_content("OK", "text/plain");
    });

    // 3. SEND STATUS
    svr.Get("/status", [](const httplib::Request&, httplib::Response& res) {
        stringstream ss;
        ss << GLOBAL_TARGET_TEMP;
        res.set_content(ss.str(), "text/plain");
    });

    cout << "Web Controller running at http://localhost:8090" << endl;
    svr.listen("0.0.0.0", 8090);
}

// --- SERIAL PORT CLASS ---
class SerialPort {
    HANDLE hSerial;
public:
    bool connected;
    SerialPort(string portName, int baudRate) {
        connected = false;
        string fullPortName = "\\\\.\\" + portName; 
        hSerial = CreateFileA(fullPortName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
        if (hSerial == INVALID_HANDLE_VALUE) return;
        DCB dcb = { 0 }; dcb.DCBlength = sizeof(dcb); GetCommState(hSerial, &dcb);
        dcb.BaudRate = baudRate; dcb.ByteSize = 8; dcb.StopBits = ONESTOPBIT; dcb.Parity = NOPARITY;
        SetCommState(hSerial, &dcb);
        
        COMMTIMEOUTS timeouts = { 0 };
        timeouts.WriteTotalTimeoutConstant = 50;
        SetCommTimeouts(hSerial, &timeouts);
        connected = true;
    }
    void write(const vector<unsigned char>& data) {
        if (!connected) return;
        DWORD bytes; WriteFile(hSerial, data.data(), data.size(), &bytes, NULL);
    }
    ~SerialPort() { if(connected) CloseHandle(hSerial); }
};

class RelayBoard {
    SerialPort* serial;
    unsigned short crc16(const vector<unsigned char>& data) {
        unsigned short crc = 0xFFFF;
        for (size_t i = 0; i < data.size(); i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) { if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; } else { crc >>= 1; } }
        } return crc;
    }
public:
    RelayBoard(SerialPort* sp) : serial(sp) {}
    void setRelay(int relayIndex, bool state) {
        vector<unsigned char> cmd = {0x01, 0x06, 0x00, (unsigned char)relayIndex, (unsigned char)(state?1:2), 0x00};
        unsigned short crc = crc16(cmd);
        cmd.push_back(crc & 0xFF); cmd.push_back((crc >> 8) & 0xFF);
        serial->write(cmd);
    }
};

class DashboardLogger {
    httplib::Client cli;
    string tokenHeader;
public:
    DashboardLogger() : cli(INFLUX_HOST, INFLUX_PORT) {
        tokenHeader = "Token "; tokenHeader += INFLUX_TOKEN;
        cli.set_connection_timeout(1);
    }
    void log(double tc1, double tc2, double setpoint, bool heaterState) {
        stringstream ss; ss << fixed << setprecision(2);
        ss << "thermal_system,unit=labjack tc1=" << tc1 << ",tc2=" << tc2 
           << ",setpoint=" << setpoint << ",heater_state=" << (heaterState ? 1 : 0);
        string path = "/api/v2/write?org="; path += INFLUX_ORG; path += "&bucket="; path += INFLUX_BUCKET; path += "&precision=s";
        httplib::Headers headers = { {"Authorization", tokenHeader}, {"Content-Type", "text/plain"} };
        auto res = cli.Post(path.c_str(), headers, ss.str(), "text/plain");
    }
};

int main() {
    // 1. START WEB SERVER
    thread webThread(RunWebServer);
    webThread.detach(); 

    // 2. SETUP HARDWARE
    SerialPort relaySerial("COM9", 9600); 
    if (!relaySerial.connected) { cout << "Relay COM Error" << endl; return 1; }
    RelayBoard relays(&relaySerial);
    DashboardLogger logger; 

    int handle; int err; int errorAddress = -1;
    err = LJM_Open(LJM_dtT7, LJM_ctANY, "ANY", &handle);
    ErrorCheck(err, "LJM_Open");
    
    // Mux80 Setup
    const char *names[] = {"AIN48_NEGATIVE_CH", "AIN48_EF_INDEX", "AIN48_EF_CONFIG_A", "AIN48_EF_CONFIG_B",
                           "AIN49_NEGATIVE_CH", "AIN49_EF_INDEX", "AIN49_EF_CONFIG_A", "AIN49_EF_CONFIG_B"};
    double values[] = {56, 24, 1, 60052,  57, 24, 1, 60052};
    LJM_eWriteNames(handle, 8, names, values, &errorAddress);

    const char *readNames[] = {"AIN48_EF_READ_A", "AIN49_EF_READ_A"};
    double readValues[2];

    cout << "Hardware Ready. Go to http://localhost:8090 to control temperature." << endl;

    // 3. MAIN LOOP
    while (true) {
        err = LJM_eReadNames(handle, 2, readNames, readValues, &errorAddress);
        double t1 = (err == 0) ? readValues[0] : -9999;
        double t2 = (err == 0) ? readValues[1] : -9999;

        double currentTarget = GLOBAL_TARGET_TEMP; 

        bool heaterState = false;
        if (t1 < currentTarget && t1 > -100.0) { 
            relays.setRelay(1, true); 
            heaterState = true;
        } else {
            relays.setRelay(1, false);
            heaterState = false;
        }

        printf("Target: %.1f C | TC1: %.2f C | TC2: %.2f C | Heater: %s   \r", 
               currentTarget, t1, t2, heaterState ? "ON " : "OFF");
        
        logger.log(t1, t2, currentTarget, heaterState);
        Sleep(1000); 
    }

    LJM_Close(handle);
    return 0;
}