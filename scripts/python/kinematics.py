import sympy as sp

# Homogeneous transformation matrices about each axis
def Rz(theta):
    return sp.Matrix([[sp.cos(theta), -sp.sin(theta), 0, 0],
                      [sp.sin(theta),  sp.cos(theta), 0, 0],
                      [0            ,  0            , 1, 0],
                      [0            ,  0            , 0, 1]])

def Ry(theta):
    return sp.Matrix([[sp.cos(theta) , 0, sp.sin(theta), 0],
                      [0             , 1, 0            , 0],
                      [-sp.sin(theta), 0, sp.cos(theta), 0],
                      [0             , 0, 0            , 1]])

def Rx(theta):
    return sp.Matrix([[1, 0            , 0             , 0],
                      [0, sp.cos(theta), -sp.sin(theta), 0],
                      [0, sp.sin(theta), sp.cos(theta) , 0],
                      [0, 0            , 0             , 1]])


def T(x, y, z):
    return sp.Matrix([[1, 0, 0, x],
                      [0, 1, 0, y],
                      [0, 0, 1, z],
                      [0, 0, 0, 1]])

# Column to extract displacement vector
D = sp.Matrix([[0],
               [0],
               [0],
               [1]])

I = sp.Matrix([[1, 0, 0, 0],
               [0, 1, 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])



# -------- Start of script -------------
sp.init_printing(use_unicode=True)

theta1, theta2, theta3 = sp.symbols('theta1 theta2 theta3')
ab, l1, l2 = sp.symbols('ab l1 l2')

a12 = Rx(theta1) * T(0, ab, 0)
a23 = Ry(theta1) * T(l1, 0, 0)
a34 = Ry(theta3) * T(l2, 0, 0)

# Forward kinematics eqn
fk = a12 * a23 * a34
fk = sp.trigsimp(fk)

# Extract the displacement vector and simplify trig
p = fk[:3,3]
px, py, pz = p

sp.pprint(px)
sp.pprint(py)
sp.pprint(pz)

