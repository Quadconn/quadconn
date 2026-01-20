import ctypes

class three_dof_theta(ctypes.Structure):
    _fields_ = [
        ("theta1", ctypes.c_double),
        ("theta2", ctypes.c_double),
        ("theta3", ctypes.c_double),
    ]

    def __str__(self):
        return f"three_dof_theta {{theta1: {self.theta1}, theta2: {self.theta2}, theta3: {self.theta3} }}"

    @staticmethod
    def type_name() -> str:
        # should have same val as IOX2_TYPE_NAME in cpp file
        return "three_dof_theta"