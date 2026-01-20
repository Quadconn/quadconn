from pathlib import Path
import sys
from sys import argv
import time
 
import pinocchio as pin
from pinocchio.visualize import GepettoVisualizer
import numpy as np
from numpy.linalg import pinv, norm


# Minimum Acceptable Distance Error (meters from target)
EPS = 1e-3

# Verify if target is within the reachable workspace of 3-DOF leg
# NOTE: This does not guarantee that the IK solver will converge 
# to the target. It is simply a quick ideal case check
def is_reachable(target):
    # Constants defining leg lengths and offset in y direction
    # TODO: Change this so that values are taken from pinocchio model (not hardcoded)
    L1 = 0.2235
    L2 = 0.19425
    L3 = 0.140
    DY = 0.04241 + 0.069
    # Origin of joint 2
    O2 = np.array([L1, DY, 0])

    # Vector to target wrt joint 2 (change of reference frame from 
    # robot origin to joint 2
    r = target - O2

    # Distance of target from the x-axis of rotation about joint 1
    rho = np.sqrt(r[1]**2 + r[2]**2)

    # By using the x and rho value we can now operate on the plane (x, rho)
    # to determine reachability

    # Hypotenuse of triangle formed by x position and rho position.
    # This is essentially the distance from the origin of joint 2 to
    # the target position
    d = np.sqrt(r[0]**2 + rho**2)

    # Check if distance is within the spherical shell shaped workspace
    # NOTE: This does not take into account joint limits ie. assumes there
    # are none
    return (abs(L2 - L3) - EPS) <= d <= (L2 + L3 + EPS)

############# URDF Model Loading / Initialization ###############

urdf_file = Path(__file__).parent.parent / "urdf/leg3dof.urdf"
 
# Load the urdf model
model = None
viz = None

if (len(argv) > 1 and argv[1] == "-v"):
    print("Visualization Enabled")

    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        urdf_file, None, None 
    )

    viz = GepettoVisualizer(model, collision_model, visual_model)
     
    # Initialize the viewer.
    try:
        viz.initViewer()
    except ImportError as err:
        print(
            "Error while initializing the viewer. "
            "It seems you should install gepetto-viewer"
        )
        print(err)
        sys.exit(0)
     
    try:
        viz.loadViewerModel("pinocchio")
    except AttributeError as err:
        print(
            "Error while loading the viewer model. "
            "It seems you should start gepetto-viewer"
        )
        print(err)
        sys.exit(0)
else:
    model = pin.buildModelFromUrdf(urdf_file)

print(f"Model Name: {model.name}")
print(f"\tnq: {model.nq}") 
print(f"\tnv: {model.nv}\n") 

# Create data required by the algorithms
data = model.createData()
 
# Sample the neutral configuration (All joints set to 0 degrees)
# TODO: Change this to a known configuration that is not neutral or random.
# Starting at neutral places leg at a singularity where IK will fail to converge
q = pin.randomConfiguration(model)
 
# Perform the forward kinematics over the frames in the kinematic tree, 
# updating 'data' in the process
pin.framesForwardKinematics(model, data, q)

if viz: 
    viz.display(q)
    time.sleep(3)

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
targets = [
        np.array([0.2235 + 0.19425 + 0.140 - 0.140 - 0.140 - 0.0001, 0.04241 + 0.069, 0.0])
]

for target in targets:
    print(f"Target position: {target}\n")
    print(f"The Target {"IS" if is_reachable(target) else "IS NOT"} Reachable")


############# Control Loop ###############

def ik(model, data, q, target, eps):
    # Integration Time Step
    DT = 1e-2
    # Proportional Control Gain
    Kp = 1.0
    # Maximum Control Loop Iterations
    MAX_IT = 1000

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
        oToTarget = target - oMtoe.translation

        # Check if error is acceptable
        if norm(oToTarget) < EPS:
            success = True
            break

        # Control law by least square 
        #   Compute joint velocity that would take the toe frame to the target (in what unit time?)
        vq = Kp * pinv(oJtoe3) @ oToTarget

        # Integrate the joint velocity over the time step and add computed configuration step 
        # to previous configuration q
        q = pin.integrate(model, q, vq * DT)

        #print(f"Error: {norm(oToTarget)}")

        i += 1

    return q, success, i

############# Results ###############
for target in targets:
    q, success, iterations = ik(model, data, q, target, EPS)
    if success:
        if (viz):
            viz.display(q)
            time.sleep(1)
        print(f"Convergence Acheived! (Iterations = {iterations})")
    else:
        print(f"Convergence Failed.")

# Compute final configuration and corresponding toe frame position
    toe_position = data.oMf[TOE_ID].translation
    print(f"Final Toe Position: {toe_position}")
    print(f"Final Error: {norm(target - toe_position)}")
    print(f"Final Configuration q: {q}")
