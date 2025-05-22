import numpy as np
import pybullet as p
import time
import matplotlib.pylab as plt

timeStep = 0.01
tMax = 15.0
p.connect(p.GUI, options="--opengl2")
p.setGravity(0.0, 0.0, -9.8)
p.setTimeOut(5.0)
p.setTimeStep(timeStep)
p.setRealTimeSimulation(False)
robot0000 = p.loadURDF("links0000.urdf", [1.0, 1.0, 1.25], useFixedBase=True)
robot9999 = p.loadURDF("pendulum.urdf", [0.0, 0.0, 1.25], useFixedBase=True)
p.resetJointState(robot0000, 0, +np.pi / 2, 0.0)
p.resetJointState(robot9999, 0, +np.pi / 2, 0.0)
p.setJointMotorControl2(bodyIndex=robot0000, jointIndex=0, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(bodyIndex=robot9999, jointIndex=0, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

nSim = int(tMax / timeStep)
data0000 = np.tile(np.nan, [nSim])
data9999 = np.tile(np.nan, [nSim])
for iTimeStep in range(nSim):
    data0000[iTimeStep] = p.getJointState(robot0000, 0)[0:2][0]
    data9999[iTimeStep] = p.getJointState(robot9999, 0)[0:2][0]
    p.stepSimulation()
    time.sleep(timeStep)
xSpace = np.linspace(0.0, tMax, nSim + 1)[0:-1:1]
p.disconnect()

plt.figure()
plt.plot(xSpace, data0000, 'o')
plt.plot(xSpace, data9999, '.')
plt.legend(['q for I = 0000', 'q for I = 9999'], loc=1)
plt.ylim([1.1 * np.min(data9999), 1.1 * np.max(data0000)])
plt.title('Bullet')
plt.xlabel('time [s]')
plt.ylabel('angle [radian]')
plt.show()































