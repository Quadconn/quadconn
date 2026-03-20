import evdev
from evdev import ecodes
import threading
import socket
import time

# Assuming GamepadData is defined in gamepad_data.py as a ctypes.Structure
from gamepad_data import GamepadData 

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


    def _monitor_loop(self):
        try:
            for event in self.device.read_loop():
                if not self.running: break
                
                if event.type == ecodes.EV_ABS:
                    val = event.value
                    code = event.code
                    if code == 0: self.left_x = (val / 32768.0) 
                    elif code == 1: self.left_y = -(val / 32768.0) # Inverted
                    elif code == 3: self.right_x = (val / 32768.0)
                    elif code == 4: self.right_y = -(val / 32768.0) # Inverted
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
            
            # Map Buttons to C Types (uint8 or int)
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

# --- Network & Usage Example ---
if __name__ == "__main__":
    # --- Network Config ---
    HOST_PC_IP = "100.81.189.79"  
    UDP_PORT = 3006
    CYCLE_TIME_SEC = 0.01         # 10ms cycle time (100 Hz)

    controller = ControllerState()
    print("Controller Interface Running...")

    # Create UDP Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Streaming data via UDP to {HOST_PC_IP}:{UDP_PORT}")

    try:
        while True:
            # 1. Fetch populated ctypes structure from the evdev reader
            data = controller.read()

            # 2. Pack it precisely into C-aligned bytes
            payload = bytes(data)

            # 3. Send over network
            sock.sendto(payload, (HOST_PC_IP, UDP_PORT))

            # Optional: Add local logic for system events here if needed, 
            # e.g., if data.Start: print("Start Pressed!")

            time.sleep(CYCLE_TIME_SEC)

    except KeyboardInterrupt:
        print("\nStopping cleanly...")
        controller.running = False
        controller.thread.join(timeout=1.0)
        sock.close()