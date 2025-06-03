import pybullet as p
import numpy as np
import os
import math
from math import pi
from .basescenario import RobotScenario, generate_terrain_from_function


def terrain_fn_A(x, y):
    return 0.1 * np.sin(0.5 * np.pi * x) * np.cos(0.5 * np.pi * y)

def terrain_fn_flat(x, y):
    return np.zeros_like(x)

class RobotThreeWheelLinearStraight(RobotScenario):
    def load(self):
        plane = generate_terrain_from_function(terrain_fn_flat, x_size=100, y_size=100, resolution=1)

        robot = p.loadURDF("urdf/robotLinearThreeWheel.urdf", basePosition=[0, 0, 1],
                           baseOrientation=p.getQuaternionFromEuler([0, 0, math.radians(90)]),
                           useFixedBase=True)
        p.changeDynamics(plane, -1, restitution=0.0, lateralFriction=1.0)
        self.body = robot
        p.getContactPoints(bodyA=robot)
        # joint name, stiffness K (Nm/rad), damping C (Nms/rad), rest position (rad), max torque
        spring_params = {
            'hipFL_to_fourcheFL': (2, 300.0, 0, 50000),
            'hipFR_to_fourcheFR': (2, 300.0, 0, 50000),
            'base_link_to_hipFL': (500.0, 1000.0, pi / 4.5, 50000),
            'base_link_to_hipFR': (500.0, 1000.0, pi / 4, 50000),
        }

        num_joints = p.getNumJoints(robot)

        for i in range(num_joints):
            info = p.getJointInfo(robot, i)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            joint_type = info[2]

            self.joint_name_to_index[joint_name] = i
            self.link_name_to_index[link_name] = i

            # Disable built-in motor
            p.setJointMotorControl2(robot, i, p.VELOCITY_CONTROL, force=0)

            if joint_type == p.JOINT_REVOLUTE:
                rest_angle = spring_params.get(joint_name, (0, 0, 0.0))[2]  # default to 0.0 if not found
                p.resetJointState(robot, i, rest_angle)


        # Build mapping from joint names to indices
        joint_name_to_index = {
            p.getJointInfo(robot, i)[1].decode(): i
            for i in range(p.getNumJoints(robot))
        }
        for joint_name, (k, c, rest, tau) in spring_params.items():
            joint_index = joint_name_to_index[joint_name]
            self.apply_spring_damper_PD(robot, joint_index, rest, k, c, tau)
        num_joints = p.getNumJoints(robot)

        #    joint name : (target speed [rad/s], P gain [Nm/(rad/s)], D gain [Nm·s/rad], max torque [Nm])
        speed_params = {
            'fourcheFL_to_wheelFL': (-5, 100.0),
            'fourcheFR_to_wheelFR': (-20.0, 100.0),
        }

        # Build mapping from joint names to indices
        joint_name_to_index = {
            p.getJointInfo(robot, i)[1].decode(): i
            for i in range(p.getNumJoints(robot))
        }

        # Now loop over each wheel joint and apply the PD speed controller
        for joint_name, (target_speed, max_torque) in speed_params.items():
            joint_index = joint_name_to_index[joint_name]
            self.apply_speed_velocity_control(robot, joint_index, target_speed, max_torque)

        # Apply dynamics settings to base link
        p.changeDynamics(
            bodyUniqueId=robot,
            linkIndex=-1,  # Base link
            mass=100.0,
            lateralFriction=1.0,
            spinningFriction=0.01,
            rollingFriction=0.01,
            restitution=0.0,
            linearDamping=0.04,
            angularDamping=0.04,
            contactStiffness=1e5,
            contactDamping=1e3,
            frictionAnchor=True
        )

        # Apply dynamics settings to all child links
        for linkIndex in range(num_joints):
            p.changeDynamics(
                bodyUniqueId=robot,
                linkIndex=linkIndex,
                mass=1.0,
                lateralFriction=1.0,
                spinningFriction=0.001,
                rollingFriction=0.001,
                restitution=0.0,
                linearDamping=0.04,
                angularDamping=0.004,
                contactStiffness=1e5,
                contactDamping=1e3,
                frictionAnchor=True
            )

    def update(self):
        pass