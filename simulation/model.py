import pybullet as p
import numpy as np
import math
from math import pi

class Model:
    """Manages the full kinematic chain, including segments and joints."""

    def __init__(self, physics_client, variant="default"):
        self.physics_client = physics_client
        self.bodies = []
        self.segments = []
        self.joints = []
        self.forces = {body: [0, 0, 0] for body in self.bodies}
        self.variant = variant
        self.joint_name_to_index = {}
        self.link_name_to_index = {}
        self.build_robot()

    def get_forces(self):
        """Retrieve forces acting on each body in the simulation."""
        for body in self.bodies:
            total_force = [0, 0, 0]

            # Iterate through all joints in the body
            num_joints = p.getNumJoints(body)
            for joint_index in range(num_joints):
                joint_state = p.getJointState(body, joint_index)
                joint_force = joint_state[2]  # Tuple (Fx, Fy, Fz)
                total_force = [total_force[i] + joint_force[i] for i in range(3)]

            # If no joints, approximate force using base velocity
            if num_joints == 0:
                lin_vel, _ = p.getBaseVelocity(body)
                total_force = list(lin_vel)  # Use velocity as an indirect measure of force

            self.forces[body] = total_force  # Store force in dictionary

        return self.forces  # Return forces for all bodies

    def build_robot(self):
        """Select the appropriate kinematic variant."""
        #self.load_surface()
        if self.variant == "default":
            self.create_robot_variant_pendulum()
        elif self.variant == "PENDULUM":
            self.create_robot_variant_pendulum()
        elif self.variant == "A":
            self.create_robot_variant_a()
        else:
            raise ValueError(f"Unknown robot variant: {self.variant}")

    def create_robot_variant_pendulum(self):
        """Creates a simple swinging pendulum using createConstraint."""
        pendulum = p.loadURDF("urdf/pendulum.urdf", basePosition=[0, 0, -0.05], useFixedBase=True)
        p.setJointMotorControl2(bodyIndex=pendulum, jointIndex=0, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)
        p.resetJointState(pendulum, 0, +np.pi / 2, 0.0)
        self.bodies.append(pendulum)

    def create_robot_variant_a(self):
        plane = p.loadURDF("urdf/plane.urdf", basePosition=[0, 0, -0.05], useFixedBase=True)
        """Loads robot, maps joints/links, disables motors, and resets joint states."""
        robot = p.loadURDF("urdf/robot.urdf", basePosition=[0, 0, 2], baseOrientation=p.getQuaternionFromEuler([0, 0, math.radians(90)]), useFixedBase=False)

        num_joints = p.getNumJoints(robot)
        for joint_index in range(num_joints):
            info = p.getJointInfo(robot, joint_index)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            joint_type = info[2]

            self.joint_name_to_index[joint_name] = joint_index
            self.link_name_to_index[link_name] = joint_index
            print(joint_index)
            # Disable motor
            p.setJointMotorControl2(
                bodyIndex=robot,
                jointIndex=joint_index,
                controlMode=p.VELOCITY_CONTROL,
                force=0
            )

            # Reset joint to 0 radians by default (you can override below)
            if joint_type in [p.JOINT_REVOLUTE]:
                p.resetJointState(robot, joint_index, 0.0)
        self.bodies.append(robot)
        self.bodies.append(plane)
        #print(self.joint_name_to_index) # {'base_joint': 0, 'base_link_to_hipFL': 1, 'hipFL_to_fourcheFL': 2, 'fourcheFL_to_wheelFL': 3, 'base_link_to_hipFR': 4, 'hipFR_to_fourcheFR': 5, 'fourcheFR_to_wheelFR': 6, 'base_link_to_hipBL': 7, 'hipBL_to_fourcheBL': 8, 'fourcheBL_to_wheelBL': 9, 'base_link_to_hipBR': 10, 'hipBR_to_fourcheBR': 11, 'fourcheBR_to_wheelBR': 12}
        #print(self.link_name_to_index) # {'base_link': 0, 'hipFL': 1, 'fourcheFL': 2, 'wheelFL': 3, 'hipFR': 4, 'fourcheFR': 5, 'wheelFR': 6, 'hipBL': 7, 'fourcheBL': 8, 'wheelBL': 9, 'hipBR': 10, 'fourcheBR': 11, 'wheelBR': 12}

    def apply_spring_damper_torque(self, robot, joint_name, k, c, rest_angle):
        # angle in radian
        # k is newton meter per radian
        # c is in radians per seconds
        joint_index = self.joint_name_to_index[joint_name]
        state = p.getJointState(robot, joint_index)
        angle = state[0]
        velocity = state[1]

        torque = -k * (angle - rest_angle) - c * velocity

        p.setJointMotorControl2(
            bodyIndex=robot,
            jointIndex=joint_index,
            controlMode=p.TORQUE_CONTROL,
            force=torque
        )

    def set_joint_position(self, joint_name, target_position):
        """Sets target joint angle (radians) for a named joint."""
        if joint_name not in self.joint_name_to_index:
            raise ValueError(f"Joint '{joint_name}' not found in robot.")
        joint_index = self.joint_name_to_index[joint_name]
        p.resetJointState(self.robot_id, joint_index, target_position)

    def print_joint_mapping(self):
        print("Joint Name → Index mapping:")
        for name, idx in self.joint_name_to_index.items():
            print(f"  {name} → {idx}")

    def print_link_mapping(self):
        print("Link Name → Index mapping:")
        for name, idx in self.link_name_to_index.items():
            print(f"  {name} → {idx}")

    def update(self):
        """Placeholder for physics update logic."""
        k = 10.0  # Nm/rad
        c = 1.0  # Nms/rad
        rest_angle = 0.0  # radians

        spring_joints = [
            'fourcheFL_to_wheelFL',
            'fourcheFR_to_wheelFR',
            'fourcheBL_to_wheelBL',
            'fourcheBR_to_wheelBR',
            'base_link_to_hipFL',
            'base_link_to_hipFR',
            'base_link_to_hipBL',
            'base_link_to_hipBR',
        ]

        spring_k = {
            'fourcheFL_to_wheelFL': 2000.0,
            'fourcheFR_to_wheelFR': 2000.0,
            'fourcheBL_to_wheelBL': 2000.0,
            'fourcheBR_to_wheelBR': 2000.0,
            'base_link_to_hipFL': 1000.0,
            'base_link_to_hipFR': 1000.0,
            'base_link_to_hipBL': 1000.0,
            'base_link_to_hipBR': 1000.0,
        }

        spring_c = {
            'fourcheFL_to_wheelFL': 100.0,
            'fourcheFR_to_wheelFR': 100.0,
            'fourcheBL_to_wheelBL': 100.0,
            'fourcheBR_to_wheelBR': 100.0,
            'base_link_to_hipFL': 10.0,
            'base_link_to_hipFR': 10.0,
            'base_link_to_hipBL': 10.0,
            'base_link_to_hipBR': 10.0,
        }

        rest_angle = {
            'fourcheFL_to_wheelFL': 0,
            'fourcheFR_to_wheelFR': 0,
            'fourcheBL_to_wheelBL': 0,
            'fourcheBR_to_wheelBR': 0,
            'base_link_to_hipFL': -pi/4,
            'base_link_to_hipFR': -pi/4,
            'base_link_to_hipBL': pi/4,
            'base_link_to_hipBR': pi/4,
        }

        for joint in spring_joints:
            self.apply_spring_damper_torque(self.bodies[0], joint, spring_k[joint], spring_c[joint], rest_angle[joint])
        pass

    def get_sensor_data(self):
        """Get the state of all links in the URDF-loaded robot."""
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

    def get_bodies(self):
        """Return the list of all bodies in the model."""
        return self.bodies


def load_model(physics_client, variant="A"):
    return Model(physics_client, variant)
