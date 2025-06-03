import pybullet as p
import numpy as np
import os

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


def generate_terrain_from_function(height_fn, x_size=20.0, y_size=20.0, resolution=0.05, z_scale=1.0, base_z=0.5):
    """
    Generates a PyBullet heightfield from a height function f(x, y).

    Args:
        height_fn (function): Function of the form f(x, y) returning float or np.array of heights.
        x_size (float): Size of terrain in X direction (meters).
        y_size (float): Size of terrain in Y direction (meters).
        resolution (float): Grid spacing (meters).
        z_scale (float): Vertical exaggeration factor.
        base_z (float): Base Z-position for the terrain object.

    Returns:
        int: PyBullet body ID of the loaded terrain.
    """
    nx = int(x_size / resolution)
    ny = int(y_size / resolution)
    xs = np.linspace(-x_size / 2, x_size / 2, nx)
    ys = np.linspace(-y_size / 2, y_size / 2, ny)
    xx, yy = np.meshgrid(xs, ys)

    zz = height_fn(xx, yy) * z_scale
    zz = zz.astype(np.float32)

    # Normalize heightfield to [0, 1]
    min_z = np.min(zz)
    max_z = np.max(zz)
    range_z = max_z - min_z if max_z > min_z else 1.0
    zz_normalized = (zz - min_z) / range_z

    # Flatten row-major for PyBullet
    heightfield_data = zz_normalized.flatten(order="C")

    terrain_shape = p.createCollisionShape(
        shapeType=p.GEOM_HEIGHTFIELD,
        meshScale=[x_size / nx, y_size / ny, range_z],
        heightfieldData=heightfield_data.tolist(),
        numHeightfieldRows=ny,
        numHeightfieldColumns=nx,
        replaceHeightfieldIndex=-1
    )

    terrain_body = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=terrain_shape,
        basePosition=[0, 0, base_z + min_z],
        baseOrientation=[0, 0, 0, 1]
    )

    return terrain_body