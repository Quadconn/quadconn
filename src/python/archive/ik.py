from pathlib import Path
import numpy as np
import iceoryx2 as iox2
from robot import Robot
import ctypes

class ThreeDoFTheta(ctypes.Structure):
    _fields_ = [
        ("theta1", ctypes.c_double),
        ("theta2", ctypes.c_double),
        ("theta3", ctypes.c_double),
    ]

    def __str__(self):
        return f"ThreeDoFTheta {{theta1: {self.theta1}, theta2: {self.theta2}, theta3: {self.theta3} }}"

    @staticmethod
    def type_name() -> str:
        # should have same val as IOX2_TYPE_NAME in cpp file
        return "ThreeDoFTheta"

class GamepadData(ctypes.Structure):
    _fields_ = [
        ("dpad_x", ctypes.c_int),
        ("dpad_y", ctypes.c_int),
        ("A", ctypes.c_int),
        ("B", ctypes.c_int),
        ("X", ctypes.c_int),
        ("Y", ctypes.c_int),
        ("Home", ctypes.c_int),
        ("Start", ctypes.c_int),
        ("Back", ctypes.c_int),
        ("L3", ctypes.c_int),
        ("R3", ctypes.c_int),
        ("lx", ctypes.c_double),
        ("ly", ctypes.c_double),
        ("rx", ctypes.c_double),
        ("ry", ctypes.c_double),
        ("RB", ctypes.c_int),
        ("RT", ctypes.c_double),
        ("LB", ctypes.c_int),
        ("LT", ctypes.c_double)
    ]

    def __str__(self):
        return f"GamepadData {{dpad_x: {self.dpad_x},  dpad_y: {self.dpad_y}, A: {self.A}, B: {self.B}, X: {self.X}, Y: {self.Y}, Home: {self.Home}, Start: {self.Start}, Back: {self.Back}, L3: {self.L3}, R3: {self.R3}, lx: {self.lx:.2f}, ly: {self.ly:.2f}, rx: {self.rx:.2f}, ry: {self.ry:.2f}, RB: {self.RB}, RT: {self.RT:.2f}, LB: {self.LB}, LT: {self.LT:.2f} }}"

    @staticmethod
    def type_name() -> str:
        return "GamepadData"

urdf_file = Path(__file__).parent.parent.parent / "urdf/leg3dof.urdf"

quad = Robot(urdf_file)
toe0 = quad.get_toe_position()
print(f"Initial Toe Position: {toe0}\n")

targets = [
        # Up 
        toe0 + np.array([0.0, 0.0, -0.08]),
        # Forward-Up
        toe0 + np.array([0.08, 0.0, -0.08]),
        # Forward-Down
        toe0 + np.array([0.08, 0.0, 0.0]),
        # Back to start
        toe0,
        # offset
        toe0 + np.array([0.08, 0.06, 0.00])
        ]
target_choice = targets[0]

if __name__ == "__main__":
    
    # iceoryx2 node publishing
    cycle_time = iox2.Duration.from_millis(10)
    iox2.set_log_level_from_env_or(iox2.LogLevel.Info)
    node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
    service = (
        node.service_builder(iox2.ServiceName.new("ThreeDoFTheta"))
        .publish_subscribe(ThreeDoFTheta)
        .open_or_create()
    )
    publisher = service.publisher_builder().create()

    service_subscriber = (
        node.service_builder(iox2.ServiceName.new("GamepadData"))
        .publish_subscribe(GamepadData)
        .open_or_create()
    )
    subscriber = service_subscriber.subscriber_builder().create()
    zero_flag = False

    # replace with continual loop whenever possible
    try:
        while True:
            node.wait(cycle_time)
            # received gamepad data values on every cycle time loop
            while True:
                received = subscriber.receive()
                if received is not None:
                    data = received.payload()
                    if (data.contents.A):
                        print("pressed A")
                        # up
                        target_choice = targets[0]
                    elif (data.contents.B):
                        print("pressed B")
                        # forward-up
                        target_choice = targets[1]
                    elif (data.contents.X):
                        print("pressed X")
                        # forward-down
                        target_choice = targets[2]
                    elif (data.contents.Y):
                        print("pressed Y")
                        # start
                        target_choice = targets[3]
                    elif (data.contents.LB):
                        target_choice = targets[4]
                    elif (data.contents.RB):
                        zero_flag = True
                    
                    print(f"Target: {target_choice}")

                    success, q = quad.leg_ik(target_choice)
                    
                    if (success):
                        print(f"Final Toe Position = {quad.get_toe_position()}")
                        print("Convergence Acheived!")
                        sample = publisher.loan_uninit()
                        if sample is not None:
                            if zero_flag:
                                sample = sample.write_payload(
                                    ThreeDoFTheta(0,0,0)
                                )
                                sample.send()
                                print(f"sent values = {q}")
                                zero_flag = False
                            else:
                                sample = sample.write_payload(
                                    ThreeDoFTheta(theta1=q[0],theta2=q[1],theta3=q[2])
                                )
                                sample.send()
                                print(f"sent values = {q}")
                        else:
                            print("could not loan memory")
                    else:
                        print("Convergence Failed!")
                    print()

                else:
                    break


    except iox2.NodeWaitFailure:
        print("failure")
