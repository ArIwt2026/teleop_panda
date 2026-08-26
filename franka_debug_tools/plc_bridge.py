import threading
import time
import sys
import snap7
from snap7.util import set_bool, get_bool

# ==========================================
# CONFIGURATION
# ==========================================
PLC_IP = "192.168.0.1"
RACK = 0
SLOT = 1

DB_NUMBER = 3
OFFSET_PLC_TO_ROS = 0    # Read from Byte 0
OFFSET_ROS_TO_PLC = 1    # Write to Byte 1

# ==========================================
# GLOBALS & THREAD SAFETY
# ==========================================
snap_mutex = threading.RLock()
keep_running = True

# ==========================================
# CORE COMMUNICATION FUNCTIONS
# ==========================================

def write_ros_to_plc(client: snap7.client.Client, bit_index: int, value: bool, log_errors: bool = True):
    with snap_mutex:
        if not client.get_connected():
            return

        try:
            # Read 1 byte at OFFSET_ROS_TO_PLC (Byte 1)
            buffer = client.db_read(DB_NUMBER, OFFSET_ROS_TO_PLC, 1)
            
            # Modify the specific bit
            set_bool(buffer, 0, bit_index, value)
            
            # Write it back
            client.db_write(DB_NUMBER, OFFSET_ROS_TO_PLC, buffer)
            
            if log_errors and bit_index != 0:
                print(f">> SUCCESS: DB{DB_NUMBER}.DBX{OFFSET_ROS_TO_PLC}.{bit_index} set to {int(value)}")
        except Exception as e:
            if log_errors:
                print(f">> [COMM ERROR] Write failed: {e}")

def print_system_status(client: snap7.client.Client):
    with snap_mutex:
        if not client.get_connected():
            print("\n>> OFFLINE: Not connected.")
            return

        try:
            # Read 2 bytes starting from 0
            buffer = client.db_read(DB_NUMBER, 0, 2)
            
            byte0 = buffer[0]
            byte1 = buffer[1]
            
            print("\n=======================================================")
            print(f"   PLC -> ROS (Byte 0) | Hex: 0x{byte0:02x} | Bits: {byte0:08b}")
            print("-------------------------------------------------------")
            print(f" [Bit 0.0] Start Storing      : {int(get_bool(buffer, 0, 0))}")
            print(f" [Bit 0.1] Start Placement    : {int(get_bool(buffer, 0, 1))}")
            print(f" [Bit 0.2] Indicator Showcase : {int(get_bool(buffer, 0, 2))}")
            print("-------------------------------------------------------")
            
            print(f"   ROS -> PLC (Byte 1) | Hex: 0x{byte1:02x} | Bits: {byte1:08b}")
            print("-------------------------------------------------------")
            print(f" [Bit 1.0] Life-Signal (Flip)   : {int(get_bool(buffer, 1, 0))}")
            print(f" [Bit 1.1] Storaging Done       : {int(get_bool(buffer, 1, 1))}")
            print(f" [Bit 1.2] Placement Done       : {int(get_bool(buffer, 1, 2))}")
            print(f" [Bit 1.3] Request empty sleigh : {int(get_bool(buffer, 1, 3))}")
            print("=======================================================\n")
            
        except Exception as e:
            print(f">> [COMM ERROR] Failed to read DB{DB_NUMBER}: {e}\n")


def heartbeat_task(client: snap7.client.Client):
    life_signal = False
    while keep_running:
        with snap_mutex:
            if not client.get_connected():
                try:
                    client.disconnect()
                    client.connect(PLC_IP, RACK, SLOT)
                except:
                    pass
            
        if client.get_connected():
            life_signal = not life_signal
            write_ros_to_plc(client, 0, life_signal, log_errors=False)
            
        time.sleep(0.5)  # 500ms sleep for 1Hz frequency

def print_menu():
    print(f"Commands:\n"
          f"  status     - Show bits in DB{DB_NUMBER}\n"
          f"  store <1/0>- Set/Clear DBX1.1\n"
          f"  place <1/0>- Set/Clear DBX1.2\n"
          f"  req <1/0>  - Set/Clear DBX1.3\n"
          f"  help       - Show menu\n"
          f"  quit       - Exit\n")

def main():
    global keep_running
    
    client = snap7.client.Client()
    
    # Optional: set timeout if python-snap7 supports it
    # client.set_param(snap7.types.PingTimeout, 2000)
    # client.set_param(snap7.types.SendTimeout, 2000)
    # client.set_param(snap7.types.RecvTimeout, 2000)

    print(f"Connecting to {PLC_IP} (Rack {RACK}, Slot {SLOT})...")
    try:
        client.connect(PLC_IP, RACK, SLOT)
        print(">> CONNECTED.")
    except Exception as e:
        print(f">> CONNECTION FAILED: {e}")

    watchdog = threading.Thread(target=heartbeat_task, args=(client,), daemon=True)
    watchdog.start()

    print_menu()

    try:
        while True:
            try:
                input_line = input("CLI> ").strip()
            except EOFError:
                break
                
            if not input_line:
                continue
                
            parts = input_line.split()
            cmd = parts[0].lower()
            
            if cmd in ["quit", "exit"]:
                break
            elif cmd == "help":
                print_menu()
            elif cmd == "status":
                print_system_status(client)
            elif cmd in ["store", "place", "req"]:
                if len(parts) > 1 and parts[1] in ["0", "1"]:
                    val = (parts[1] == "1")
                    if cmd == "store":
                        write_ros_to_plc(client, 1, val)
                    elif cmd == "place":
                        write_ros_to_plc(client, 2, val)
                    elif cmd == "req":
                        write_ros_to_plc(client, 3, val)
    except KeyboardInterrupt:
        pass
    finally:
        keep_running = False
        watchdog.join(timeout=1.0)
        client.disconnect()
        client.destroy()

if __name__ == "__main__":
    main()
