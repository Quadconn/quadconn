import pybullet as p
import pybullet_data
import sys

from quad_ipc import QuadIpcSubscriber, QuadIpcError
import joint_angles
import quad_common

# NOTE: If you want to only visualize run this script as "python sim.py -v"

if __name__ == "__main__":

    isVisualizeOnly = False
    if len(sys.argv) > 1 and sys.argv[1] == "-v":
        isVisualizeOnly = True

    subIpc = QuadIpcSubscriber("joint_angles", joint_angles.BodyJointAngles)

    # Load model
    physicsClient = p.connect(p.GUI)

    if not isVisualizeOnly:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeId = p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -9.8)

    startPos = [0,0,1]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    legId = p.loadURDF("../../urdf/basic_quad.urdf",startPos, startOrientation, useFixedBase=isVisualizeOnly)
    jointIndicies = [i for i in range(p.getNumJoints(legId))]
    # Let physics server auto step the simulation
    p.setRealTimeSimulation(True)

    try:
        while True:
            subIpc.wait(10)
            while True:

                data = subIpc.receive()

                if data is not None:

                    q = []
                    for i in range(quad_common.LEG_COUNT):
                        q.extend([data.contents.body_joint_angles[i].hip_roll,
                                  data.contents.body_joint_angles[i].hip_pitch,
                                  data.contents.body_joint_angles[i].knee_pitch])

                    p.setJointMotorControlArray(legId, jointIndicies, controlMode=p.POSITION_CONTROL, targetPositions=q)

                else:
                    break


    except QuadIpcError as e:
        print(e);
        p.disconnect()
