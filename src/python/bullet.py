import pybullet as p
import time
import numpy as np


physicsClient = p.connect(p.GUI)

startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
legId = p.loadURDF("../../urdf/leg3dof.urdf",startPos, startOrientation, useFixedBase=1)

p.setJointMotorControl2(legId, 2, controlMode=p.POSITION_CONTROL, targetPosition=-np.pi/2)

for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)


p.disconnect()

