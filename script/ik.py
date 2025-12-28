from pathlib import Path
 
import pinocchio as pin
from numpy.linalg import pinv, norm
import numpy as np
 
urdf_file = Path(__file__).parent.parent / "urdf/leg3dof.urdf"
 
# Load the urdf model
model = pin.buildModelFromUrdf(urdf_file)
print(f"Model Name: {model.name}")
 
# Create data required by the algorithms
data = model.createData()
 
# Sample the neutral configuration (All joints set to 0 degrees)
q = pin.neutral(model)
print(f"Neutral Configuration q: {q.T}")
 
# Perform the forward kinematics over the kinematic tree, updating 'data' in the process
pin.forwardKinematics(model, data, q)
# Update the frame placements with the new data after forward kinematics
pin.updateFramePlacements(model, data)

# Get the frame index/id of the robots toe frame
toe_frame_id = model.getFrameId("toe")
# Get the actual frame placement of the toe (rotation & translation)
toe_pose = data.oMf[toe_frame_id]
# Extract only the translation
toe_position = toe_pose.translation

# At this point with a neutral configuration the toe is positioned at a point
# when the leg is fully extended in the x direction and has some y translation
# due to the L shaped links being attached to each other
print(f"Toe position: {toe_position}")

# Define a target with a slight offset in -x from where the toe is positioned
#target = toe_position + np.array([-0.05, 0.0, -0.05])
target = np.array([0.2235, 0.11141, 0.33425])
print(f"Target position: {target}")

# Integration Time Step
DT = 1e-2

# Control Gain
Kp = 1.0

# Minimum Acceptable Error
EPS = 1e-2

# Control loop
for i in range(500):
    # Perform forward kinematics on robots frames and compute jacobians
    pin.framesForwardKinematics(model, data, q)
    pin.computeJointJacobians(model, data, q)

    # Current placement of toe frame
    oMtoe = data.oMf[toe_frame_id]

    # 3D jacobian (only linear/translation components)
    oJtoe3 = pin.computeFrameJacobian(model, data, q, toe_frame_id, pin.LOCAL_WORLD_ALIGNED)[:3,:]

    # Distance error vector from current toe position to target
    oToTarget = oMtoe.translation - target

    # Check if error is acceptable
    if norm(oToTarget) < EPS:
        print(f"Convergence Acheived! (Iterations = {i})")
        break

    # Control law by least square 
    #   Compute joint velocity that would take the toe frame to the target (in what unit time?)
    vq = -Kp * pinv(oJtoe3) @ oToTarget

    # Integrate the joint velocity over the time step and add computed configuration step 
    # to previous configuration q
    q = pin.integrate(model, q, vq * DT)

    #print(f"Error: {norm(oToTarget)}")

# Compute final configuration and corresponding toe frame position
pin.framesForwardKinematics(model, data, q)
toe_position = data.oMf[toe_frame_id].translation
print("Final Toe position:", toe_position)
print(f"Final Configuration q: {q.T}")
