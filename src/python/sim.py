import pybullet as p

from quad_ipc import QuadIpcSubscriber, QuadIpcError
from joint_angles import JointAngles

# NOTE: Currently only using this as a visualizer with no 
# gravity so I am not using p.setGravity() yet

if __name__ == "__main__":

    subIpc = QuadIpcSubscriber("joint_angles", JointAngles)

    # Load model
    physicsClient = p.connect(p.GUI)
    startPos = [0,0,0]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    legId = p.loadURDF("../../urdf/leg3dof.urdf",startPos, startOrientation, useFixedBase=True)
    jointIndicies = [i for i in range(p.getNumJoints(legId) - 1)]
    # Let physics server auto step the simulation
    p.setRealTimeSimulation(True)

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


    except QuadIpcError as e:
        print(e);
        p.disconnect()
