import ctypes
import socket
import time
import os

# Hide the Pygame welcome message
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

from gamepad_data import GamepadData

class WindowsControllerState:
    def __init__(self):
        # Initialize Pygame and the Joystick module in the main thread
        pygame.init()
        pygame.joystick.init()
        
        if pygame.joystick.get_count() == 0:
            raise IOError("No gamepad found! Please connect a controller.")
            
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"[OK] Gamepad found: {self.joystick.get_name()}")

    def read(self) -> GamepadData:
        """
        Reads the current state from XInput via Pygame and returns the ctypes structure.
        """
        # CRITICAL FIX: Pump the event queue in the main thread right before reading!
        pygame.event.pump()

        # --- Axes ---
        lx = self.joystick.get_axis(0)
        ly = -self.joystick.get_axis(1)
        rx = self.joystick.get_axis(2)
        ry = -self.joystick.get_axis(3)
        
        # Map triggers (convert -1.0 -> 1.0 range to 0.0 -> 1.0)
        lt_raw = self.joystick.get_axis(4) if self.joystick.get_numaxes() > 4 else -1.0
        rt_raw = self.joystick.get_axis(5) if self.joystick.get_numaxes() > 5 else -1.0
        l2 = (lt_raw + 1.0) / 2.0
        r2 = (rt_raw + 1.0) / 2.0

        # --- D-Pad ---
        dpad_x, dpad_y = 0, 0
        if self.joystick.get_numhats() > 0:
            hat = self.joystick.get_hat(0)
            dpad_x = hat[0]
            dpad_y = hat[1]

        # --- Buttons ---
        btn = lambda idx: self.joystick.get_button(idx) if idx < self.joystick.get_numbuttons() else 0

        # Standard Pygame Xbox button mapping
        # 0:A, 1:B, 2:X, 3:Y, 4:LB, 5:RB, 6:Select, 7:Start, 8:L3, 9:R3, 10:Guide
        return GamepadData(
            dpad_x=int(dpad_x),
            dpad_y=int(dpad_y),
            A=btn(0),
            B=btn(1),
            X=btn(2),
            Y=btn(3),
            LB=btn(4),
            RB=btn(5),
            Select=btn(6),
            Start=btn(7),
            L3=btn(8),
            R3=btn(9),
            Home=btn(10),
            lx=lx,
            ly=ly,
            rx=rx,
            ry=ry,
            LT=l2,
            RT=r2
        )

# --- Network & Usage Example ---
if __name__ == "__main__":
    # --- Network Config ---
    HOST_PC_IP = "100.81.189.79"  
    UDP_PORT = 3006
    CYCLE_TIME_SEC = 0.01         # 10ms cycle time (100 Hz)

    try:
        controller = WindowsControllerState()
        print("Controller Interface Running...")
    except IOError as e:
        print(e)
        exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"Streaming data via UDP to {HOST_PC_IP}:{UDP_PORT}")

    try:
        while True:
            #  Fetch populated ctypes structure
            data = controller.read()
            
            # Print to terminal so you can verify the sticks/buttons are working
            # print(f"\r{data}", end="") 
            
            #  Pack it into bytes
            payload = bytes(data)
        
            #  Send over network
            sock.sendto(payload, (HOST_PC_IP, UDP_PORT))

            time.sleep(CYCLE_TIME_SEC)

    except KeyboardInterrupt:
        print("\nStopping cleanly...")
        sock.close()
        pygame.quit()