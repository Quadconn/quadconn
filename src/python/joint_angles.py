import ctypes

import quad_common

class LegJointAngles(ctypes.Structure):
    _fields_ = [
        ("hip_roll", ctypes.c_double),
        ("hip_pitch", ctypes.c_double),
        ("knee_pitch", ctypes.c_double)
    ]

    def __str__(self) -> str:
        return f"LegJointAngles {{ hip_roll: {self.hip_roll}, hip_pitch: {self.hip_pitch}, knee_pitch: {self.knee_pitch} }}"

    @staticmethod
    def type_name() -> str:
        # should have same val as IOX2_TYPE_NAME in cpp file
        return "LegJointAngles"

class BodyJointAngles(ctypes.Structure):
    _fields_ = [
        ("body_joint_angles", LegJointAngles * quad_common.LEG_COUNT),
    ]

    @staticmethod
    def type_name() -> str:
        # should have same val as IOX2_TYPE_NAME in cpp file
        return "BodyJointAngles"
