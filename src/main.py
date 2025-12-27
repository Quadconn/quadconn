from pathlib import Path
from sys import argv
from numpy.linalg import pinv,inv,norm,svd,eig
import numpy as np
 
import pinocchio
 
# This path refers to Pinocchio source code but you can define your own directory here.
pinocchio_model_dir = Path(__file__).parent.parent / "models"
 
# You should change here to set up your own URDF file or just pass it as an argument of
# this example.
urdf_filename = (
    pinocchio_model_dir / "example-robot-data/robots/ur_description/urdf/ur5_robot.urdf"
    if len(argv) < 2
    else argv[1]
)
 
# Load the urdf model
model = pinocchio.buildModelFromUrdf(urdf_filename)
print("model name: " + model.name)
 
# Create data required by the algorithms
data = model.createData()
 
# Sample a random configuration
q = pinocchio.randomConfiguration(model)
print(f"q: {q.T}")
 
# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model, data, q)
 
# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))


DT = 1e-3
IDX_TOOL = model.getFrameId('foot')
print(f"Foot frame: {IDX_TOOL}")

oMgoal = pinocchio.SE3(pinocchio.Quaternion(-0.5, 0.58, -0.39, 0.52).normalized().matrix(), np.array([1.2, .4, .7]))

# Loop on an inverse kinematics for 200 iterations.
for i in range(500):  # Integrate over 2 second of robot life

    # Run the algorithms that outputs values in robot.data
    pinocchio.framesForwardKinematics(model,data,q)
    pinocchio.computeJointJacobians(model,data,q)

    # Placement from world frame o to frame f oMtool
    oMtool = data.oMf[IDX_TOOL]

    # 3D jacobian in world frame
    o_Jtool3 = pinocchio.computeFrameJacobian(model,data,q,IDX_TOOL,pinocchio.LOCAL_WORLD_ALIGNED)[:3,:]

    # vector from tool to goal, in world frame
    o_TG = oMtool.translation-oMgoal.translation
    
    # Control law by least square
    vq = -10*pinv(o_Jtool3)@o_TG

    q = pinocchio.integrate(model,q, vq * DT)
