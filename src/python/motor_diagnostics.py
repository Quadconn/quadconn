from ctypes import Structure, c_int, c_double, sizeof

MOTOR_COUNT = 12

class MotorDiagnostics(Structure):
    """
    This structure maps the raw binary memory from the C++ side.
    The order of these fields must be IDENTICAL to the C++ struct.
    """
    _fields_ = [
        ("mode", c_int),
        ("fault", c_int),
        ("trajectory_complete", c_int),
        ("position", c_double),
        ("velocity", c_double),
        ("torque", c_double),
        ("q_current", c_double),
        ("d_current", c_double),
        ("abs_position", c_double),
        ("power", c_double),
        ("motor_temperature", c_double),
        ("voltage", c_double),
        ("temperature", c_double),
    ]

class MotorDiagnosticsArray(Structure):
    """A wrapper for the 12 motors being sent as a single UDP packet."""
    _fields_ = [
        ("motor_instance", MotorDiagnostics * MOTOR_COUNT),
    ]

class MotorInfo:
    """Utility functions to turn numbers into readable text."""
    
    @staticmethod
    def mode_to_string(mode: int) -> str:
        modes = {
            0: "STOP",
            1: "FAULT",
            2: "ENABLING",
            3: "CALIB",
            10: "POS",
            15: "BRAKE",
        }
        return modes.get(mode, f"MODE {mode}")

    @staticmethod
    def fault_to_string(fault: int) -> str:
        if fault == 0:
            return "OK"
        faults = {
            34: "OVER_VOLT",
            38: "OVER_TEMP",
            40: "UNDER_VOLT",
            44: "ENABLE_FLT"
        }
        return faults.get(fault, f"FLT {fault}")