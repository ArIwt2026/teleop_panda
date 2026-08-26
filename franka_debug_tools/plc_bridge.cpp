#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <iomanip>
#include <bitset>
#include "snap7.h"

using namespace std;

// ==========================================
// CONFIGURATION
// ==========================================
const char* PLC_IP = "192.168.0.1"; 
const int RACK = 0;
const int SLOT = 1;                 

const int DB_NUMBER = 3;           
const int OFFSET_PLC_TO_ROS = 0;    // Read from Byte 0
const int OFFSET_ROS_TO_PLC = 1;    // Write to Byte 1

// ==========================================
// GLOBALS & THREAD SAFETY
// ==========================================
std::recursive_mutex snap_mutex;           
std::atomic<bool> keep_running(true);

// ==========================================
// HELPER FUNCTIONS
// ==========================================
bool getBit(uint8_t byte, int bit) { 
    return (byte & (1 << bit)) != 0; 
}

void setBit(uint8_t& byte, int bit, bool value) {
    if (value) byte |= (1 << bit);
    else       byte &= ~(1 << bit);
}

string get_error_text(int error_code) {
    char buf[1024];
    Cli_ErrorText(error_code, buf, 1024);
    stringstream ss;
    ss << buf << " (Code: 0x" << hex << error_code << dec << ")";
    return ss.str();
}

string to_hex(uint8_t* buf, int len) {
    stringstream ss;
    for (int i = 0; i < len; i++) {
        ss << "0x" << setfill('0') << setw(2) << hex << (int)buf[i] << " ";
    }
    return ss.str();
}

// ==========================================
// CORE COMMUNICATION FUNCTIONS
// ==========================================

void write_ros_to_plc(TS7Client* client, int bit_index, bool value, bool log_errors = true) {
    std::lock_guard<std::recursive_mutex> lock(snap_mutex); 
    if (!client->Connected()) return;

    uint8_t buffer[1] = {0};
    int read_res = client->DBRead(DB_NUMBER, OFFSET_ROS_TO_PLC, 1, &buffer);
    if (read_res != 0) {
        if (log_errors) cout << ">> [COMM ERROR] Cannot read Byte " << OFFSET_ROS_TO_PLC << ": " << get_error_text(read_res) << "\n";
        return;
    }

    setBit(buffer[0], bit_index, value);
    int write_res = client->DBWrite(DB_NUMBER, OFFSET_ROS_TO_PLC, 1, &buffer);
    if (write_res != 0) {
        if (log_errors) cout << ">> [COMM ERROR] Write failed: " << get_error_text(write_res) << "\n";
    } else if (log_errors && bit_index != 0) {
         cout << ">> SUCCESS: DB" << DB_NUMBER << ".DBX" << OFFSET_ROS_TO_PLC << "." << bit_index << " set to " << value << "\n";
    }
}

// Spawns a detached thread to set the bit to 1, sleep 1000ms, and set to 0.
void pulse_ros_to_plc(TS7Client* client, int bit_index) {
    write_ros_to_plc(client, bit_index, true, true);
    std::thread([client, bit_index]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        if (keep_running) {
            write_ros_to_plc(client, bit_index, false, false); 
            cout << ">> (Pulse Ended: DB" << DB_NUMBER << ".DBX" << OFFSET_ROS_TO_PLC << "." << bit_index << " reset to 0)\nCLI> ";
            cout.flush();
        }
    }).detach();
}

void print_system_status(TS7Client* client) {
    std::lock_guard<std::recursive_mutex> lock(snap_mutex);
    if (!client->Connected()) {
        cout << "\n>> OFFLINE: Not connected.\n";
        return;
    }

    uint8_t buf[2] = {0, 0};
    int res = client->DBRead(DB_NUMBER, 0, 2, &buf);

    cout << "\n=======================================================\n";
    if (res == 0) {
        cout << "   PLC -> ROS (Byte 0) | Hex: " << to_hex(&buf[0], 1) << " | Bits: " << bitset<8>(buf[0]) << "\n";
        cout << "-------------------------------------------------------\n";
        cout << " [Bit 0.0] LifeBit 1Hz        : " << getBit(buf[0], 0) << "\n";
        cout << " [Bit 0.1] StartStoraging     : " << getBit(buf[0], 1) << "\n";
        cout << " [Bit 0.2] StartPlacement     : " << getBit(buf[0], 2) << "\n";
        cout << " [Bit 0.3] IndicatorShowcase  : " << getBit(buf[0], 3) << "\n";
        cout << " [Bit 0.4] Start Recycling    : " << getBit(buf[0], 4) << "\n";
        cout << " [Bit 0.5] Assembly Mode      : " << getBit(buf[0], 5) << " (1=Assembly, 0=Disassembly)\n";
        cout << "-------------------------------------------------------\n";
        cout << "   ROS -> PLC (Byte 1) | Hex: " << to_hex(&buf[1], 1) << " | Bits: " << bitset<8>(buf[1]) << "\n";
        cout << "-------------------------------------------------------\n";
        cout << " [Bit 1.0] LifeBit 1Hz (Flip) : " << getBit(buf[1], 0) << "\n";
        cout << " [Bit 1.1] Storaging Done     : " << getBit(buf[1], 1) << "\n";
        cout << " [Bit 1.2] Placement Done     : " << getBit(buf[1], 2) << "\n";
        cout << " [Bit 1.3] REQ empty sleigh   : " << getBit(buf[1], 3) << "\n";
        cout << " [Bit 1.4] Recycling Done     : " << getBit(buf[1], 4) << "\n";
    } else {
        cout << ">> [COMM ERROR] Failed to read DB" << DB_NUMBER << ": " << get_error_text(res) << "\n";
    }
    cout << "=======================================================\n\n";
}

void heartbeat_task(TS7Client* client) {
    bool life_signal = false;
    auto last_changed_time = std::chrono::steady_clock::now();
    bool last_incoming_lifebit = false;
    bool first_read = true;

    while (keep_running) {
        bool connected = false;
        {
            std::lock_guard<std::recursive_mutex> lock(snap_mutex);
            connected = client->Connected();
        }

        if (!connected) {
            std::lock_guard<std::recursive_mutex> lock(snap_mutex);
            if (!client->Connected()) {
                client->Disconnect(); 
                client->ConnectTo(PLC_IP, RACK, SLOT);
                first_read = true;
            }
        }

        if (client->Connected()) {
            // Watchdog check on incoming LifeBit
            uint8_t buf[1] = {0};
            int res;
            {
                std::lock_guard<std::recursive_mutex> lock(snap_mutex);
                res = client->DBRead(DB_NUMBER, OFFSET_PLC_TO_ROS, 1, &buf);
            }

            if (res == 0) {
                bool incoming_lifebit = getBit(buf[0], 0);
                if (first_read || incoming_lifebit != last_incoming_lifebit) {
                    last_incoming_lifebit = incoming_lifebit;
                    last_changed_time = std::chrono::steady_clock::now();
                    first_read = false;
                } else {
                    auto now = std::chrono::steady_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_changed_time).count();
                    if (duration > 2500) {
                        cout << "\n>> [WATCHDOG] PLC LifeBit stalled (>2.5s)! Connection may be dead. Forcing reconnect...\nCLI> ";
                        cout.flush();
                        std::lock_guard<std::recursive_mutex> lock(snap_mutex);
                        client->Disconnect();
                        first_read = true;
                        continue; 
                    }
                }
            }

            // Write outgoing LifeBit
            life_signal = !life_signal; 
            write_ros_to_plc(client, 0, life_signal, false); 
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void print_menu() {
    cout << "Commands:\n"
         << "  status  - Show bits in DB" << DB_NUMBER << "\n"
         << "  store   - Pulse Storaging Done (DBX1.1)\n"
         << "  place   - Pulse Placement Done (DBX1.2)\n"
         << "  req <1/0>- Set/Clear REQ empty sleigh (DBX1.3)\n"
         << "  recycle - Pulse Recycling Done (DBX1.4)\n"
         << "  help    - Show menu\n"
         << "  quit    - Exit\n\n";
}

int main() {
    TS7Client* client = new TS7Client();
    int timeout = 2000;
    client->SetParam(6, &timeout); 
    client->SetParam(7, &timeout); 

    cout << "Connecting to " << PLC_IP << " (Rack " << RACK << ", Slot " << SLOT << ")...\n";
    int res = client->ConnectTo(PLC_IP, RACK, SLOT);
    if (res == 0) cout << ">> CONNECTED.\n";
    else cout << ">> CONNECTION FAILED: " << get_error_text(res) << "\n";

    std::thread watchdog(heartbeat_task, client);
    print_menu();

    string input_line;
    while (true) {
        cout << "CLI> ";
        if (!getline(cin, input_line)) break;
        stringstream ss(input_line);
        string cmd;
        ss >> cmd;
        if (cmd.empty()) continue;
        if (cmd == "quit" || cmd == "exit") break;
        else if (cmd == "help") print_menu();
        else if (cmd == "status") print_system_status(client);
        else if (cmd == "store")   pulse_ros_to_plc(client, 1);
        else if (cmd == "place")   pulse_ros_to_plc(client, 2);
        else if (cmd == "req") {
            string val_str;
            ss >> val_str;
            if (val_str == "1") write_ros_to_plc(client, 3, true);
            else if (val_str == "0") write_ros_to_plc(client, 3, false);
            else cout << "Usage: req <1/0>\n";
        }
        else if (cmd == "recycle") pulse_ros_to_plc(client, 4);
        else cout << "Unknown command.\n";
    }

    keep_running = false;
    watchdog.join();
    client->Disconnect();
    delete client;
    return 0;
}