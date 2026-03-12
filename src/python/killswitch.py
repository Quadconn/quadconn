import iceoryx2 as iox2
import quad_ipc

from gamepad_data import GamepadData, GamepadEnums
from gamepad_data import to_event_id, from_event_id

# --- Usage Example ---
if __name__ == "__main__":
    print("starting killswitch reader")
    
    # iceoryx2 node publishing
    
    iox2.set_log_level_from_env_or(iox2.LogLevel.Info)

    node = quad_ipc.make_node("killswitch")
    # subscriber = quad_ipc.make_subscriber(quad_ipc.make_service("GamepadData", GamepadData, node))
    event = (
        node
        .service_builder(iox2.ServiceName.new("GamepadData"))
        .event()
        .open_or_create()
    )
    listener = event.listener_builder().create()
    try:
        while True:
            event_id = listener.blocking_wait_one()
            if event_id == to_event_id(GamepadEnums.Start):
                print("start")
            if event_id == to_event_id(GamepadEnums.Select):
                print("select")
            
    except KeyboardInterrupt:
        print("\nStopping cleanly...")
