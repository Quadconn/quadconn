from gamepad_data import GamepadData
import iceoryx2 as iox2
import quad_ipc


# --- Usage Example ---
if __name__ == "__main__":
    print("starting killswitch reader")
    
    # iceoryx2 node publishing
    cycle_time = iox2.Duration.from_millis(250)
    iox2.set_log_level_from_env_or(iox2.LogLevel.Info)

    node = quad_ipc.make_node("killswitch")
    subscriber = quad_ipc.make_subscriber(quad_ipc.make_service("GamepadData", GamepadData, node))

    try:
        while True:
            node.wait(cycle_time)
            
            read_data = quad_ipc.ipc_receive(subscriber)
            
            
            # debug remove later
            
            
    except KeyboardInterrupt:
        print("\nStopping cleanly...")
