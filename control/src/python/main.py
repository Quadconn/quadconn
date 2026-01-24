from pathlib import Path
import numpy as np
import time

from robot import Robot, RobotVisualizer

urdf_file = Path(__file__).parent.parent.parent / "urdf/leg3dof.urdf"

quad = Robot(urdf_file)
quadViz = RobotVisualizer(quad)
toe0 = quad.get_toe_position()
print(f"Initial Toe Position: {toe0}\n")

targets = [
        # Up 
        toe0 + np.array([0.0, 0.0, -0.04]),
        # Forward-Up
        toe0 + np.array([0.04, 0.0, -0.04]),
        # Forward-Down
        toe0 + np.array([0.04, 0.0, 0.0]),
        # Back to start
        toe0,
        ]

quadViz.display(quad)
time.sleep(5)
for t in targets:
    print(f"Target: {t}")

    success, q = quad.leg_ik(t)

    if (success):
        quadViz.display(quad)
        time.sleep(1)
        print(f"Final Toe Position = {quad.get_toe_position()}")
        print("Convergence Acheived!")
        print(f"q = {q}")
    else:
        print("Convergence Failed!")
    print()
