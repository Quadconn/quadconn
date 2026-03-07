import ctypes
# --- The C-Compatible Data Structure ---
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
        ("Back", ctypes.c_int),
        ("L3", ctypes.c_int),
        ("R3", ctypes.c_int),
        ("RB", ctypes.c_int),
        ("LB", ctypes.c_int)
    ]

    def __str__(self):
        return f"GamepadData {{dpad_x: {self.dpad_x},  dpad_y: {self.dpad_y}, A: {self.A}, B: {self.B}, X: {self.X}, Y: {self.Y}, Home: {self.Home}, Start: {self.Start}, Back: {self.Back}, L3: {self.L3}, R3: {self.R3}, lx: {self.lx:.2f}, ly: {self.ly:.2f}, rx: {self.rx:.2f}, ry: {self.ry:.2f}, RB: {self.RB}, RT: {self.RT:.2f}, LB: {self.LB}, LT: {self.LT:.2f} }}"

    @staticmethod
    def type_name() -> str:
        return "GamepadData"
