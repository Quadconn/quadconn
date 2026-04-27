import ctypes
import socket
import iceoryx2 as iox2
import quad_ipc
from system_logic import SystemLogic, to_event_id
# --- The C-Compatible Data Structure ---
# This MUST match the sender exactly.
class GamepadData(ctypes.Structure):
    _fields_ = [
        ("lx", ctypes.c_double),
        ("ly", ctypes.c_double),
        ("rx", ctypes.c_double),
        ("ry", ctypes.c_double),
        ("dpad_x", ctypes.c_int), 
        ("dpad_y", ctypes.c_int), 
        ("RT", ctypes.c_double),
        ("LT", ctypes.c_double),
        ("A", ctypes.c_int),
        ("B", ctypes.c_int),
        ("X", ctypes.c_int),
        ("Y", ctypes.c_int),
        ("Home", ctypes.c_int),
        ("Start", ctypes.c_int),
        ("Select", ctypes.c_int),
        ("L3", ctypes.c_int),
        ("R3", ctypes.c_int),
        ("RB", ctypes.c_int),
        ("LB", ctypes.c_int)
    ]

    # Required by iceoryx2 to identify the data type across the system
    @staticmethod
    def type_name() -> str:
        return "GamepadData"

def main():
    # --- UDP Configuration ---
    UDP_IP = "0.0.0.0"  # Listen on all available network interfaces
    UDP_PORT = 3006
    struct_size = ctypes.sizeof(GamepadData)

    # --- iceoryx2 Node Setup ---
    print("Initializing iceoryx2 Bridge Node...")
    iox2.set_log_level_from_env_or(iox2.LogLevel.Info)
    
    # Create the Node
    node = quad_ipc.make_node("udp_receive")
    
    # Setup the Publisher Service for the Gamepad Data
    data_service = (
        node.service_builder(iox2.ServiceName.new("GamepadData"))
        .publish_subscribe(GamepadData)
        .open_or_create()
    )
    publisher = data_service.publisher_builder().create()

    # Setup the Event Notifier Service for System Commands (Start/Stop)
    event_service = (
        node.service_builder(iox2.ServiceName.new("SystemLogic"))
        .event()
        .open_or_create()
    )
    notifier = event_service.notifier_builder().create() 

    # --- UDP Socket Setup ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    print(f"Listening for UDP packets on port {UDP_PORT} (Expected size: {struct_size} bytes)...")
    print("Bridging UDP data to iceoryx2 IPC. Press Ctrl+C to stop.")

    try:
        while True:

            node.wait(iox2.Duration.from_millis(10))
            # 1. Receive data from the Edge PC over UDP
            # recvfrom is a blocking call, so the loop naturally paces itself 
            # to the incoming 100Hz network stream.
            payload, addr = sock.recvfrom(1024)
            
            # 2. Validate packet size to prevent memory alignment crashes
            if len(payload) == struct_size:
                
                # Cast raw bytes directly back into the ctypes structure
                data = GamepadData.from_buffer_copy(payload)

                # 3. Publish to iceoryx2 shared memory
                sample = publisher.loan_uninit()
                if sample is not None:
                    sample = sample.write_payload(data)
                    sample.send()
                else:
                    print("Warning: Could not loan shared memory from iceoryx2")

                # REMOVED KILL_MOTORS (look at control code for that)
                if data.Start:
                    notifier.notify_with_custom_event_id(
                        to_event_id(SystemLogic.StartMotors))
            else:
                print(f"Warning: Received packet of {len(payload)} bytes, expected {struct_size}.")
                
    except KeyboardInterrupt:
        print("\nShutting down bridge node cleanly...")
    finally:
        sock.close()

if __name__ == "__main__":
    main()