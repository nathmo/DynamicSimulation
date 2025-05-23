import pybullet as p
import numpy as np
import math
from math import pi

# Base class for all scenarios
class RobotScenario:
    def __init__(self, physics_client):
        self.physics_client = physics_client
        self.bodies = []
        self.joint_name_to_index = {}
        self.link_name_to_index = {}

    def load(self):
        raise NotImplementedError

    def update(self):
        pass

    def get_sensor_data(self):
        data = {}
        robot_id = self.bodies[0]  # Assuming first entry is the robot
        num_joints = p.getNumJoints(robot_id)

        for link_index in range(-1, num_joints):  # -1 is the base
            if link_index == -1:
                pos, orn = p.getBasePositionAndOrientation(robot_id)
                lin_vel, ang_vel = p.getBaseVelocity(robot_id)
            else:
                state = p.getLinkState(robot_id, link_index, computeLinkVelocity=1)
                pos, orn = state[0], state[1]
                lin_vel, ang_vel = state[6], state[7]

            data[link_index] = {
                "position": pos,
                "velocity": lin_vel,
                "orientation": orn,
            }

        return data

    def get_forces(self):
        """
        Retrieve net joint reaction forces acting on each body in the simulation.
        Returns a dictionary mapping body unique IDs to [Fx, Fy, Fz] force vectors.
        """
        self.forces = {}

        for body in self.bodies:
            total_force = np.array([0.0, 0.0, 0.0])
            num_joints = p.getNumJoints(body)

            if num_joints > 0:
                for joint_index in range(num_joints):
                    joint_state = p.getJointState(body, joint_index)
                    reaction_forces = joint_state[2][:3]  # Only Fx, Fy, Fz
                    total_force += np.array(reaction_forces)
            else:
                # Fallback: use linear velocity as a proxy (not a real force)
                lin_vel, _ = p.getBaseVelocity(body)
                total_force = np.array(lin_vel)

            self.forces[body] = total_force.tolist()

        return self.forces

    def get_bodies(self):
        return self.bodies

# Scenario: Pendulum
class PendulumScenario(RobotScenario):
    def load(self):
        pendulum = p.loadURDF("urdf/pendulum.urdf", basePosition=[0, 0, -0.05], useFixedBase=True)
        p.setJointMotorControl2(pendulum, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
        p.resetJointState(pendulum, 0, np.pi / 2, 0.0)
        self.bodies.append(pendulum)

# Scenario: Robot A
class RobotAScenario(RobotScenario):
    def load(self):
        plane = p.loadURDF("urdf/plane.urdf", basePosition=[0, 0, -0.05], useFixedBase=True)
        robot = p.loadURDF("urdf/robot.urdf", basePosition=[0, 0, 2],
                           baseOrientation=p.getQuaternionFromEuler([0, 0, math.radians(90)]),
                           useFixedBase=False)

        num_joints = p.getNumJoints(robot)
        for i in range(num_joints):
            info = p.getJointInfo(robot, i)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            joint_type = info[2]
            self.joint_name_to_index[joint_name] = i
            self.link_name_to_index[link_name] = i

            p.setJointMotorControl2(robot, i, p.VELOCITY_CONTROL, force=0)
            if joint_type == p.JOINT_REVOLUTE:
                p.resetJointState(robot, i, 0.0)

        self.bodies += [robot, plane]

    def apply_spring_damper_torque(self, robot, joint_name, k, c, rest_angle):
        joint_index = self.joint_name_to_index[joint_name]
        angle, velocity = p.getJointState(robot, joint_index)[:2]
        torque = -k * (angle - rest_angle) - c * velocity
        p.setJointMotorControl2(robot, joint_index, p.TORQUE_CONTROL, force=torque)

    def update(self):
        spring_params = {
            'fourcheFL_to_wheelFL': (2000.0, 100.0, 0),
            'fourcheFR_to_wheelFR': (2000.0, 100.0, 0),
            'fourcheBL_to_wheelBL': (2000.0, 100.0, 0),
            'fourcheBR_to_wheelBR': (2000.0, 100.0, 0),
            'base_link_to_hipFL': (1000.0, 10.0, -pi/4),
            'base_link_to_hipFR': (1000.0, 10.0, -pi/4),
            'base_link_to_hipBL': (1000.0, 10.0, pi/4),
            'base_link_to_hipBR': (1000.0, 10.0, pi/4),
        }

        robot = self.bodies[0]
        for joint, (k, c, rest) in spring_params.items():
            self.apply_spring_damper_torque(robot, joint, k, c, rest)

# Top-level Model that wraps scenario
class Model:
    scenario_classes = {
        "default": PendulumScenario,
        "PENDULUM": PendulumScenario,
        "A": RobotAScenario,
    }

    def __init__(self, physics_client, variant="default"):
        if variant not in self.scenario_classes:
            raise ValueError(f"Unknown variant: {variant}")

        self.scenario = self.scenario_classes[variant](physics_client)
        self.scenario.load()

    def update(self):
        self.scenario.update()

    def get_sensor_data(self):
        return self.scenario.get_sensor_data()

    def get_forces(self):
        return self.scenario.get_forces()

    def get_bodies(self):
        return self.scenario.get_bodies()

    def print_joint_mapping(self):
        print("Joint Name → Index mapping:")
        for name, idx in self.scenario.joint_name_to_index.items():
            print(f"  {name} → {idx}")

    def print_link_mapping(self):
        print("Link Name → Index mapping:")
        for name, idx in self.scenario.link_name_to_index.items():
            print(f"  {name} → {idx}")

# Loader function
def load_model(physics_client, variant="A"):
    return Model(physics_client, variant)
