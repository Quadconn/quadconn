import sympy as sp

# Rotation matrices 
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

# Translation matrix
def T(x, y, z):
    return sp.Matrix([[1, 0, 0, x],
                      [0, 1, 0, y],
                      [0, 0, 1, z],
                      [0, 0, 0, 1]])


# NOTE: calculations are for a left leg!

if __name__ == "__main__":
    sp.init_printing(use_unicode=True)

    theta1, theta2, theta3 = sp.symbols('theta1 theta2 theta3')
    # Abduction length, thigh length, knee length
    ab, l1, l2 = sp.symbols('ab l1 l2')

    # Hip roll
    a12 = Rx(theta1) * T(0, ab, 0)
    # Hip pitch
    a23 = Ry(theta2) * T(0, 0, -l1)
    # Knee pitch
    a34 = Ry(theta3) * T(0, 0, -l2)

    sp.pprint(a12)
    sp.pprint(a23)
    sp.pprint(a34)

    # Forward kinematics eqn
    fk = a12 * a23 * a34
    fk = sp.trigsimp(fk)

    # Extract the xyz equations and simplify trig
    p = fk[:3,3]
    px, py, pz = p

    sp.pprint(px)
    sp.pprint(py)
    sp.pprint(pz)
