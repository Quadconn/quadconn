# System Startup and Service Management Procedures

This document outlines the services used in starting up quadconn.
To enable the services, navigate to the directory with the services 
and place them onto the miniPC. 
---

```bash
sudo cp -a services/* /etc/systemd/system/
```

## 1. Enabling and Starting Core Services

The system relies on these services:
* `controller_udp.service`: **STARTUP**--Manages controller input via UDP.
* `udp_lidar.service`: **STARTUP**--Sends Lidar Data via UDP.
* `zero_motors.service`: **STARTUP**--Start-Up Script; set angular positions of motor.
* `udp_diagnostics.service`: --Sends UDP motor diagnostic data via UDP.
* `motor_interface.service`: Handles direct communication with the motor hardware.
* `controls.service`: Computes the kinematic positions and sends them to the motor hardware.


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