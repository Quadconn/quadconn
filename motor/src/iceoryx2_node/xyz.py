import ctypes

class target_coords(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double),
    ]

    def __str__(self):
        return f"target_coords {{x: {self.x}, y: {self.y}, z: {self.z} }}"

    @staticmethod
    def type_name() -> str:
        # should have same val as IOX2_TYPE_NAME in cpp file
        return "target_coords"