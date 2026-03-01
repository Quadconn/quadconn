import pybullet as p

from quad_ipc import QuadIpcSubscriber, QuadIpcError
import joint_angles
import quad_common

# NOTE: Currently only using this as a visualizer with no 
# gravity so I am not using p.setGravity() yet

if __name__ == "__main__":

    subIpc = QuadIpcSubscriber("joint_angles", joint_angles.BodyJointAngles)

    # Load model
    physicsClient = p.connect(p.GUI)
    startPos = [0,0,0]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    legId = p.loadURDF("../../urdf/basic_quad.urdf",startPos, startOrientation, useFixedBase=True)
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
                        #print(f"received {i}: ", data.contents.body_joint_angles[i])
                        q.extend([data.contents.body_joint_angles[i].hip_roll,
                                  data.contents.body_joint_angles[i].hip_pitch,
                                  data.contents.body_joint_angles[i].knee_pitch])

                    print(len(q), len(jointIndicies))

                    p.setJointMotorControlArray(legId, jointIndicies, controlMode=p.POSITION_CONTROL, targetPositions=q)

                else:
                    break


    except QuadIpcError as e:
        print(e);
        p.disconnect()
