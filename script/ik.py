from pathlib import Path
from sys import argv
 
import pinocchio as pin
import numpy as np
from numpy.linalg import pinv, norm


############# URDF Model Loading / Initialization ###############
 
urdf_file = Path(__file__).parent.parent / "urdf/leg3dof.urdf"
 
# Load the urdf model
model = pin.buildModelFromUrdf(urdf_file)
print(f"Model Name: {model.name}\n")
 
# Create data required by the algorithms
data = model.createData()
 
# Sample the neutral configuration (All joints set to 0 degrees)
q = pin.neutral(model)
 
# Perform the forward kinematics over the frames in the kinematic tree, 
# updating 'data' in the process
pin.framesForwardKinematics(model, data, q)

# Get the frame index/id of the robots toe frame
TOE_ID = model.getFrameId("toe")
# Get the actual frame placement of the toe (rotation & translation)
toe_pose = data.oMf[TOE_ID]
# Extract only the translation
toe_position = toe_pose.translation

# At this point with a neutral configuration the toe is positioned at a point
# when the leg is fully extended in the x direction and has some y translation
# due to the L shaped links being attached to each other
print(f"Neutral Toe position: {toe_position}")

# Define a target with a slight offset in -x from where the toe is positioned
#target = toe_position + np.array([-0.05, 0.0, -0.05])
target = np.array([0.2235, 0.11141, 0.33425])
print(f"Target position: {target}\n")


############# Control Loop Constants ###############

# Integration Time Step
DT = 1e-2

# Proportional Control Gain
Kp = 1.0

# Minimum Acceptable Distance Error (meters from target)
EPS = 1e-2

# Maximum Control Loop Iterations
MAX_IT = 1000


############# Control Loop ###############
i = 0
success = False
while (True):

    if (i > MAX_IT):
        print(f"Maximum Iterations Reached ({i})")
        break

    # Perform forward kinematics on robots frames and compute jacobians
    pin.framesForwardKinematics(model, data, q)
    pin.computeJointJacobians(model, data, q)

    # Current placement of toe frame
    oMtoe = data.oMf[TOE_ID]

    # 3D jacobian (only linear/translation components)
    oJtoe3 = pin.computeFrameJacobian(model, data, q, TOE_ID, pin.LOCAL_WORLD_ALIGNED)[:3,:]

    # Distance error vector from current toe position to target
    oToTarget = oMtoe.translation - target

    # Check if error is acceptable
    if norm(oToTarget) < EPS:
        success = True
        break

    # Control law by least square 
    #   Compute joint velocity that would take the toe frame to the target (in what unit time?)
    vq = -Kp * pinv(oJtoe3) @ oToTarget

    # Integrate the joint velocity over the time step and add computed configuration step 
    # to previous configuration q
    q = pin.integrate(model, q, vq * DT)

    #print(f"Error: {norm(oToTarget)}")

    i += 1


############# Results ###############

if success:
    print(f"Convergence Acheived! (Iterations = {i})")
else:
    print(f"Convergence Failed.")

# Compute final configuration and corresponding toe frame position
toe_position = data.oMf[TOE_ID].translation
print(f"Final Toe position: {toe_position}")
print(f"Final Error: {norm(toe_position - target)}")
