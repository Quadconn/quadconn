import socket
import time
import ctypes
from motor_diagnostics import MotorDiagnosticsArray

# Target: Your Mac (Localhost)
UDP_IP = "100.119.158.85"
UDP_PORT = 808

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_scenario(name, voltage, power, fault_motor_idx=None):
    print(f"--- SCENARIO: {name} ---")
    telem = MotorDiagnosticsArray()
    
    for i in range(12):
        telem.motor_instance[i].voltage = voltage
        telem.motor_instance[i].power = power / 12  # Split total power across motors
        telem.motor_instance[i].position = 0.0
        
        # Inject a fault if specified
        if fault_motor_idx is not None and i == fault_motor_idx:
            telem.motor_instance[i].fault = 34 # Over-volt fault code
        else:
            telem.motor_instance[i].fault = 0

    # Convert the C-struct to raw bytes and send
    sock.sendto(bytes(telem), (UDP_IP, UDP_PORT))

try:
    while True:
        # 1. HEALTHY STATE
        # 16.0V, 50W total draw, No faults
        send_scenario("HEALTHY", 16.0, 50.0)
        time.sleep(3)

        # 2. HIGH POWER WARNING
        # 15.8V, 350W total (Should trigger Amber Banner)
        send_scenario("HIGH POWER", 15.8, 350.0)
        time.sleep(3)

        # 3. MOTOR FAULT
        # 15.5V, 60W, Motor 4 has a fault (Should trigger Red Banner + Pop-up)
        send_scenario("MOTOR FAULT", 15.5, 60.0, fault_motor_idx=3)
        time.sleep(3)

        # 4. CRITICAL BATTERY
        # 14.2V (Should trigger Red Banner + Battery Pop-up)
        send_scenario("LOW BATTERY", 14.2, 40.0)
        time.sleep(3)

except KeyboardInterrupt:
    print("Test stopped.")
