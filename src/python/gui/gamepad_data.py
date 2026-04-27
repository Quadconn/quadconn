import ctypes

class GamepadData(ctypes.Structure):
    _fields_ = [
        ("lx", ctypes.c_double),
        ("ly", ctypes.c_double),
        ("rx", ctypes.c_double),
        ("ry", ctypes.c_double),
        ("dpad_x", ctypes.c_int), 
        ("dpad_y", ctypes.c_int), 
        ("RT", ctypes.c_double),
        ("LT", ctypes.c_double),
        ("A", ctypes.c_int),
        ("B", ctypes.c_int),
        ("X", ctypes.c_int),
        ("Y", ctypes.c_int),
        ("Home", ctypes.c_int),
        ("Start", ctypes.c_int),
        ("Select", ctypes.c_int),
        ("L3", ctypes.c_int),
        ("R3", ctypes.c_int),
        ("RB", ctypes.c_int),
        ("LB", ctypes.c_int)
    ]