import evdev
from evdev import ecodes
import threading
import iceoryx2 as iox2
from gamepad_data import GamepadData
from system_logic import SystemLogic, to_event_id





# --- The Controller Logic ---
class ControllerState:
    # Standard Linux Xbox Button Codes
    BTN_CONSTANTS = {
        'A': 304, 'B': 305, 'X': 307, 'Y': 308,
        'LB': 310, 'RB': 311,
        'LS_CLK': 317, 'RS_CLK': 318,
        'SELECT': 314, 'START': 315, 'GUIDE': 316
    }

    def __init__(self, device_path=None):
        self.device = self._find_device(device_path)
        self.running = True
        
        # Internal State
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.l2 = 0.0
        self.r2 = 0.0
        self.dpad_x = 0
        self.dpad_y = 0
        self.buttons = set()

        # Start Thread
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()

    def _find_device(self, path=None):
        if path:
            return evdev.InputDevice(path)

        devices = [evdev.InputDevice(p) for p in evdev.list_devices()]

        for d in devices:
            name = d.name.lower()
            caps = d.capabilities()

            # Xbox / gamepad detection
            if (
                ecodes.EV_ABS in caps and
                ecodes.EV_KEY in caps and
                (
                    "xbox" in name or
                    "x-box" in name or
                    "360" in name or
                    "controller" in name or
                    "wireless receiver" in name
                )
            ):
                print(f"[OK] Gamepad found: {d.path} -> {d.name}")
                return d

        # Debug dump
        print("\n--- Available input devices ---")
        for d in devices:
            print(d.path, "->", d.name)
        print("------------------------------\n")

        raise IOError("No gamepad found.")

    def _deadzone(self, joystick_input):
        return 0.0 if abs(joystick_input) < 0.05 else joystick_input

    def _monitor_loop(self):
        try:
            for event in self.device.read_loop():
                if not self.running: break
                
                if event.type == ecodes.EV_ABS:
                    val = event.value
                    code = event.code
                    # Flipping all joystick axes to follow typical x-y plot signs
                    if code == 0: self.left_x = self._deadzone(-val / 32768.0)
                    elif code == 1: self.left_y = self._deadzone(-val / 32768.0)
                    elif code == 3: self.right_x = self._deadzone(-val / 32768.0)
                    elif code == 4: self.right_y = self._deadzone(-val / 32768.0)
                    elif code == 2: self.l2 = val / 255.0
                    elif code == 5: self.r2 = val / 255.0
                    elif code == 16: self.dpad_x = val
                    elif code == 17: self.dpad_y = -val
                    
                elif event.type == ecodes.EV_KEY:
                    if event.value == 1: self.buttons.add(event.code)
                    elif event.value == 0: self.buttons.discard(event.code)
        except OSError: pass

    def read(self) -> GamepadData:
        """
        Returns a populated gamepad_data ctypes structure.
        """
        # Helper lambda to check button state (returns 1 or 0)
        btn = lambda name: 1 if self.BTN_CONSTANTS[name] in self.buttons else 0

        return GamepadData(
            dpad_x=int(self.dpad_x),
            dpad_y=int(self.dpad_y),
            
            # Map Buttons to C Types (uint8)
            A=btn('A'),
            B=btn('B'),
            X=btn('X'),
            Y=btn('Y'),
            Home=btn('GUIDE'),
            Start=btn('START'),
            Select=btn('SELECT'),
            L3=btn('LS_CLK'),
            R3=btn('RS_CLK'),
            LB=btn('LB'),
            RB=btn('RB'),

            # Map Axes to C Types (double)
            lx=self.left_x,
            ly=self.left_y,
            rx=self.right_x,
            ry=self.right_y,
            LT=self.l2,
            RT=self.r2
        )

# --- Usage Example ---
if __name__ == "__main__":
    controller = ControllerState()
    print("Controller Interface Running...")
    
    # iceoryx2 node publishing
    cycle_time = iox2.Duration.from_millis(10)
    iox2.set_log_level_from_env_or(iox2.LogLevel.Info)
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    service = (
        node.service_builder(iox2.ServiceName.new("GamepadData"))
        .publish_subscribe(GamepadData)
        .open_or_create()
    )
    publisher = service.publisher_builder().create()

    # create event-based node
    start_motors = iox2.EventId.new(1)
    stop_motors = iox2.EventId.new(2)

    event_service = (
        node
        .service_builder(iox2.ServiceName.new("SystemLogic"))
        .event()
        .open_or_create()
    )

    notifier = event_service.notifier_builder().create() 



    try:

        prev_data = controller.read()
        while True:
            node.wait(cycle_time)

            # initialize data to make notifiers
            sample = publisher.loan_uninit()
            data = controller.read()
            if sample is not None:
                sample = sample.write_payload(
                    data 
                )
                sample.send()
                # print(f"\r{data}", end="")
            else:
                print("could not loan memory")
            
            # notifiers (REMOVED KILLMOTORS)
            if data.Start:
                notifier.notify_with_custom_event_id(
                    to_event_id(SystemLogic.StartMotors))


            prev_data = data
            # debug remove later
            
            
    except KeyboardInterrupt:
        print("\nStopping cleanly...")
        controller.running = False
