import pinocchio as pin
import numpy as np
from   numpy.linalg import pinv, norm

import pathlib

# TODO: Come back and replace any return values or passed in values that 
# should be copied instead of holding a reference to (mostly np arrays)
class Robot:
    # Integration Time Step
    _DT = 1e-2
    # Proportional Control Gain
    _KP = 1.0
    # Maximum Control Loop Iterations
    _MAX_IT = 1000
    # Minimum Acceptable Distance Error (meters from target)
    _EPS = 1e-3

    # Initial robot configuration
    _INITIAL_Q = np.array([np.deg2rad(0), np.deg2rad(-135), np.deg2rad(90)])

    def __init__(self, urdf_path: pathlib.Path, q0: np.ndarray | None = None):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()
        self.q     = q0 if q0 is not None else self._INITIAL_Q 
        self.toe_id = self.model.getFrameId("toe")

        # Update models data based on q
        pin.framesForwardKinematics(self.model, self.data, self.q)

    def leg_ik(self, target: np.ndarray) -> tuple[bool, np.ndarray]:
        i = 0
        success = False

        while (True):
            if (i > self._MAX_IT):
                break

            # Perform forward kinematics on robots frames and compute jacobians
            pin.framesForwardKinematics(self.model, self.data, self.q)
            pin.computeJointJacobians(self.model, self.data, self.q)

            # Current placement of toe frame
            oMtoe = self.data.oMf[self.toe_id]

            # 3D jacobian (only linear/translation components)
            oJtoe3 = pin.computeFrameJacobian(self.model, self.data, self.q, self.toe_id, pin.LOCAL_WORLD_ALIGNED)[:3,:]

            # Distance error vector from current toe position to target
            oToTarget = target - oMtoe.translation

            # Check if error is acceptable
            if norm(oToTarget) < self._EPS:
                success = True
                break

            # Control law by least square 
            #   Compute joint velocity that would take the toe frame to the target (in what unit time?)
            vq = self._KP * pinv(oJtoe3) @ oToTarget

            # Integrate the joint velocity over the time step and add computed configuration step 
            # to previous configuration q
            self.q = pin.integrate(self.model, self.q, vq * self._DT)

            i += 1

        # Call before exit to ensure data is up to date
        pin.framesForwardKinematics(self.model, self.data, self.q)

        return success, self.q

    def get_toe_position(self) -> np.ndarray:
        return self.data.oMf[self.toe_id].translation

