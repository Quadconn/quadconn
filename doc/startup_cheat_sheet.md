# System Startup and Service Management Procedures

This document outlines the procedures for managing the systemd services that control the robotic subsystems, including enabling core processes, handling the motor startup sequence, and configuring services to run automatically on boot.

---

## 1. Enabling and Starting Core Services

The system relies on these services:
* `motor_interface.service`: Handles direct communication with the motor hardware.
* `controls.service`: Computes the kinematic positions and sends them to the motor hardware.
* `udp_controller.service`: Manages controller input via UDP.
* `udp_diagnostics.service`: Sends UDP motor diagnostic data via UDP.
* `udp_lidar.service`: Sends Lidar Data via UDP.
* `zero_motors.service`: Start-Up Script; set angular positions of motor.
* `unfold.service`: Mechanical script; moves the robot upright from a sitting position.

**To enable these services (allowing them to be started):**
You must reload the systemd daemon anytime you create or modify these files, and then enable them.
```bash
sudo systemctl daemon-reload
sudo systemctl enable motor_interface.service controls.service udp_controller.service
```

Verify if the services are running using 
```bash
sudo systemctl status motor_interface.service controls.service udp_controller.service
```