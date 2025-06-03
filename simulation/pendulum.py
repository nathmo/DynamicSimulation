import pybullet as p
import numpy as np
import os
import math
from math import pi
from .basescenario import RobotScenario

# Scenario: Pendulum
class PendulumScenario(RobotScenario):
    def load(self):
        pendulum = p.loadURDF("urdf/pendulum.urdf", basePosition=[0, 0, -0.05], useFixedBase=True)
        p.setJointMotorControl2(pendulum, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
        p.resetJointState(pendulum, 0, np.pi / 2, 0.0)
        self.body = pendulum