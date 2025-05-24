import pybullet as p
import numpy as np
import math
from math import pi

# Base class for all scenarios
class RobotScenario:
    def __init__(self, physics_client, time_step):
        self.physics_client = physics_client
        self.body = None
        self.time_step = time_step
        self.joint_name_to_index = {}
        self.link_name_to_index = {}
        self.previous_velocities = {}  # For acceleration calculation

    def load(self):
        raise NotImplementedError

    def update(self):
        pass

    def get_sensor_data(self):
        """
        Returns a dictionary:
        {
            link_name: {
                "position": ...,
                "velocity": ...,
                "acceleration": ...,
                "orientation": ...,
                "force": ...
            },
            ...
        }
        """
        data = {}
        body = self.body
        num_joints = p.getNumJoints(body)

        # Base link: name it "base" or get from somewhere (base has no joint name)
        base_name = "base"

        pos, orn = p.getBasePositionAndOrientation(body)
        lin_vel, _ = p.getBaseVelocity(body)
        prev_vel = self.previous_velocities.get(base_name, [0.0, 0.0, 0.0])
        acc = (np.array(lin_vel) - np.array(prev_vel)) / self.time_step
        self.previous_velocities[base_name] = lin_vel

        base_force = sum(np.array(p.getJointState(body, j)[2][:3]) for j in range(num_joints))

        data[base_name] = {
            "position": pos,
            "velocity": lin_vel,
            "acceleration": acc.tolist(),
            "orientation": orn,
            "force": base_force.tolist()
        }

        # Links by name
        for link_idx in range(num_joints):
            joint_info = p.getJointInfo(body, link_idx)
            link_name = joint_info[1].decode('utf-8')  # decode bytes to string

            state = p.getLinkState(body, link_idx, computeLinkVelocity=1)
            pos, orn = state[0], state[1]
            lin_vel = state[6]

            prev_vel = self.previous_velocities.get(link_name, [0.0, 0.0, 0.0])
            acc = (np.array(lin_vel) - np.array(prev_vel)) / self.time_step
            self.previous_velocities[link_name] = lin_vel

            force = p.getJointState(body, link_idx)[2][:3]

            data[link_name] = {
                "position": pos,
                "velocity": lin_vel,
                "acceleration": acc.tolist(),
                "orientation": orn,
                "force": list(force)
            }

        return data

    def get_list_of_segment(self):
        segments = []
        num_links = p.getNumJoints(self.body)

        # Base segment name
        segments.append("base")

        # Add all link names
        for link_index in range(num_links):
            joint_info = p.getJointInfo(self.body, link_index)
            link_name = joint_info[1].decode('utf-8')
            segments.append(link_name)

        return segments

    def apply_spring_damper_PD(self, robot, joint_index, target_position, k, c, tau_max):
        """
        Applies a PD-based spring-damper control using PyBullet's internal POSITION_CONTROL.

        Args:
            robot (int): PyBullet body ID.
            joint_index (int): Index of the joint to control.
            target_position (float): Desired angular position (in radians).
            k (float): Torsional spring constant (Nm/rad).
            c (float): Damping coefficient (Nms/rad).
            tau_max (float): Max torque to apply at the joint (Nm).
        """
        # Map physical parameters to PyBullet's dimensionless gains:
        # positionGain = k / tau_max
        # velocityGain = c / tau_max
        position_gain = k / tau_max
        velocity_gain = c / tau_max

        p.setJointMotorControl2(
            bodyUniqueId=robot,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_position,
            positionGain=position_gain,
            velocityGain=velocity_gain,
            force=tau_max
        )

    def apply_spring_damper_torque(self, robot, joint_name, k, c, rest_angle):
        joint_index = self.joint_name_to_index[joint_name]
        angle, velocity = p.getJointState(robot, joint_index)[:2]
        torque = -k * (angle - rest_angle) - c * velocity
        p.setJointMotorControl2(robot, joint_index, p.TORQUE_CONTROL, force=torque)

    def apply_speed_velocity_control(self, robot, joint_index, target_speed, max_torque):
        """
        Applies velocity control to a revolute joint (wheel).

        Args:
            robot (int): The PyBullet body unique ID.
            joint_index (int): The index of the joint to control.
            target_speed (float): Desired angular velocity (rad/s).
            max_torque (float): Maximum allowable torque (Nm).
        """
        # Let PyBullet handle the velocity control
        p.setJointMotorControl2(
            bodyIndex=robot,
            jointIndex=joint_index,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=target_speed,
            force=max_torque  # Limit the max torque the motor can apply
        )


# Scenario: Pendulum
class PendulumScenario(RobotScenario):
    def load(self):
        pendulum = p.loadURDF("urdf/pendulum.urdf", basePosition=[0, 0, -0.05], useFixedBase=True)
        p.setJointMotorControl2(pendulum, 0, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
        p.resetJointState(pendulum, 0, np.pi / 2, 0.0)
        self.body = pendulum

# Scenario: Robot A
class RobotAScenario(RobotScenario):
    def load(self):
        plane = p.loadURDF("urdf/plane_flat.urdf", basePosition=[0, 0, -0.05], useFixedBase=True)

        robot = p.loadURDF("urdf/robot.urdf", basePosition=[0, 0, 0.2],
                           baseOrientation=p.getQuaternionFromEuler([0, 0, math.radians(90)]),
                           useFixedBase=True)
        p.changeDynamics(plane, -1, restitution=0.0, lateralFriction=1.0)
        self.body = robot
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

        # joint name, stiffness K (Nm/rad), damping C (Nms/rad), rest position (rad), max torque
        spring_params = {
            'fourcheFL_to_wheelFL': (2000.0, 100.0, 0, 50000),
            'fourcheFR_to_wheelFR': (2000.0, 100.0, 0, 50000),
            'fourcheBL_to_wheelBL': (2000.0, 100.0, 0, 50000),
            'fourcheBR_to_wheelBR': (2000.0, 100.0, 0, 50000),
            'base_link_to_hipFL': (500.0, 1000.0, -pi / 4, 50000),
            'base_link_to_hipFR': (500.0, 1000.0, -pi / 4, 50000),
            'base_link_to_hipBL': (500.0, 1000.0, pi / 4, 50000),
            'base_link_to_hipBR': (500.0, 1000.0, pi / 4, 50000),
        }

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
            'fourcheFL_to_wheelFL': (10.0, 100.0),
            'fourcheFR_to_wheelFR': (10.0, 100.0),
            'fourcheBL_to_wheelBL': (10.0, 100.0),
            'fourcheBR_to_wheelBR': (10.0, 100.0),
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
            mass=1.0,
            lateralFriction=1.0,
            spinningFriction=0.1,
            rollingFriction=0.1,
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
        # joint name, K , c, rest position
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


# Top-level Model that wraps scenario
class Model:
    scenario_classes = {
        "default": PendulumScenario,
        "PENDULUM": PendulumScenario,
        "A": RobotAScenario,
    }

    def __init__(self, physics_client, time_step, variant="default"):
        if variant not in self.scenario_classes:
            raise ValueError(f"Unknown variant: {variant}")
        self.time_step = time_step
        self.scenario = self.scenario_classes[variant](physics_client, self.time_step)
        self.scenario.load()

    def update(self):
        self.scenario.update()

    def get_sensor_data(self):
        return self.scenario.get_sensor_data()

    def get_list_of_segment(self):
        return self.scenario.get_list_of_segment()

    def print_joint_mapping(self):
        print("Joint Name → Index mapping:")
        for name, idx in self.scenario.joint_name_to_index.items():
            print(f"  {name} → {idx}")

    def print_link_mapping(self):
        print("Link Name → Index mapping:")
        for name, idx in self.scenario.link_name_to_index.items():
            print(f"  {name} → {idx}")

# Loader function
def load_model(physics_client, variant="A", time_step=1/120):
    return Model(physics_client, time_step, variant)
