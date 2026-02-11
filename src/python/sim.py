import pybullet as p
import ctypes

from quad_ipc import QuadIpcSubscriber, QuadIpcError

class JointAngles(ctypes.Structure):
    _fields_ = [
        ("hip_roll", ctypes.c_double),
        ("hip_pitch", ctypes.c_double),
        ("knee_pitch", ctypes.c_double)
    ]

    def __str__(self) -> str:
        return f"JointAngles {{ hip_roll: {self.hip_roll}, hip_pitch: {self.hip_pitch}, knee_pitch: {self.knee_pitch} }}"

    @staticmethod
    def type_name() -> str:
        # should have same val as IOX2_TYPE_NAME in cpp file
        return "JointAngles"


if __name__ == "__main__":

    subIpc = QuadIpcSubscriber("joint_angles", JointAngles)

    # Load model
    physicsClient = p.connect(p.GUI)
    startPos = [0,0,0]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    legId = p.loadURDF("../../urdf/leg3dof.urdf",startPos, startOrientation, useFixedBase=1)
    jointIndicies = [i for i in range(p.getNumJoints(legId) - 1)]
    # Let physics server auto step the simulation
    p.setRealTimeSimulation(1)

    try:
        while True:
            subIpc.wait(10)
            while True:
                data = subIpc.receive()
                if data is not None:
                    print("received: ", data.contents)

                    q = [data.contents.hip_roll, data.contents.hip_pitch, data.contents.knee_pitch]
                    p.setJointMotorControlArray(legId, jointIndicies, controlMode=p.POSITION_CONTROL, targetPositions=q)

                else:
                    break


    except QuadIpcError:
        p.disconnect()
        print("exit");
