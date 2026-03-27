import pybullet as p
import pybullet_data
import sys

import iceoryx2 as iox2
import quad_ipc
import joint_angles
import quad_common

# NOTE: If you want to only visualize run this script as "python sim.py -v"

JOINT_MAX_FORCE = 3.0
GRAVITY = -9.8

if __name__ == "__main__":

    isVisualizeOnly = False
    if len(sys.argv) > 1 and sys.argv[1] == "-v":
        isVisualizeOnly = True

    iox2.set_log_level_from_env_or(iox2.LogLevel.Info)
    cycle_time = iox2.Duration.from_millis(quad_common.DT_MILLI)
    sim_node = quad_ipc.make_node("sim_node")
    joint_subscriber = (
        quad_ipc.make_subscriber(quad_ipc.make_service("BodyJointAngles", joint_angles.BodyJointAngles, sim_node))
    )

    # Load model
    physicsClient = p.connect(p.GUI)

    if not isVisualizeOnly:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        planeId = p.loadURDF("plane.urdf")
        p.setGravity(0, 0, GRAVITY)

    startPos = [0,0,1]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    legId = p.loadURDF("../../urdf/quad/urdf/quad.urdf",startPos, startOrientation, useFixedBase=isVisualizeOnly)
    jointIndicies = [i for i in range(p.getNumJoints(legId))]
    max_forces = [JOINT_MAX_FORCE] * len(jointIndicies)

    # Let physics server auto step the simulation
    p.setRealTimeSimulation(True)

    while True:
        sim_node.wait(cycle_time)

        data = quad_ipc.ipc_receive(joint_subscriber) 

        if data is not None:

            q = []
            for i in range(quad_common.LEG_COUNT):
                q.extend([data.contents.body_joint_angles[i].hip_roll,
                          data.contents.body_joint_angles[i].hip_pitch,
                          data.contents.body_joint_angles[i].knee_pitch])

            p.setJointMotorControlArray(legId, jointIndicies, controlMode=p.POSITION_CONTROL, targetPositions=q, forces=max_forces)

