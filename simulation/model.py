import pybullet as p
import numpy as np


class Model:
    def __init__(self, physics_client, variant="default"):
        self.physics_client = physics_client
        self.bodies = []
        self.variant = variant
        self.build_robot()
        self.forces = {body: [0, 0, 0] for body in self.bodies}  # Initialize forces dictionary

    def build_robot(self):
        if self.variant == "default":
            self.create_robot_default()
        elif self.variant == "A":
            self.create_robot_variant_a()
        else:
            raise ValueError(f"Unknown robot variant: {self.variant}")

    def create_robot_default(self):
        # Load the floor from the URDF file (output_mesh/plane.urdf)
        plane_id = p.loadURDF("output_mesh/plane.urdf", basePosition=[0, 0, -0.05], useFixedBase=True)
        self.bodies.append(plane_id)

        # Define the rectangular body (10 kg mass, 2m length, 0.6m width, 0.1m thickness)
        rectangle_length = 2.0  # 2m
        rectangle_width = 0.6  # 60 cm
        rectangle_thickness = 0.1  # 10 cm thickness
        rectangle_mass = 10.0  # 10 kg mass

        # Create the rectangle (box shape)
        rectangle_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[rectangle_length / 2, rectangle_width / 2,
                                                                          rectangle_thickness / 2])

        # Create the multi-body for the rectangle (placed on the ground)
        rectangle_id = p.createMultiBody(
            baseMass=rectangle_mass,
            baseCollisionShapeIndex=rectangle_shape,
            basePosition=[0, 0, rectangle_thickness / 2]  # Center at z = thickness/2
        )
        self.bodies.append(rectangle_id)

        # Define wheel parameters (70 cm diameter, axis aligned with width)
        wheel_diameter = 0.7  # 70 cm
        wheel_radius = wheel_diameter / 2

        # Define positions of the wheels relative to the center of the rectangle
        wheel_positions = [
            [-1, rectangle_width / 2, 0],  # A
            [1, rectangle_width / 2, 0],  # B
            [-1, -rectangle_width / 2, 0],  # C
            [1, -rectangle_width / 2, 0]  # D
        ]

        # Create the wheels and add them to the bodies
        for pos in wheel_positions:
            wheel = self.create_wheel(pos, wheel_radius, rectangle_id)
            self.bodies.append(wheel)

    def create_wheel(self, position, radius, parent_id):
        # Create a wheel (cylinder shape) with the rotation axis aligned with the width of the rectangle (along x-axis)
        wheel_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=0.1)  # Thin wheels

        # Rotation of 90 degrees around the y-axis to align the wheel's rotation axis with the x-axis
        orientation = p.getQuaternionFromEuler([0, np.pi / 2, 0])  # Rotate 90 degrees around the y-axis

        # Create wheel multi-body
        wheel_id = p.createMultiBody(
            baseMass=1.0,  # Assuming 1 kg for each wheel
            baseCollisionShapeIndex=wheel_shape,
            basePosition=position,
            baseOrientation=orientation  # Apply the rotation
        )

        # Attach the wheel to the rectangle using a fixed joint
        p.createConstraint(
            parentBodyUniqueId=parent_id,  # Rectangle body
            parentLinkIndex=-1,  # Base of the rectangle (no specific link, it's the body itself)
            childBodyUniqueId=wheel_id,  # Wheel body
            childLinkIndex=-1,  # Base of the wheel (no specific link)
            jointType=p.JOINT_FIXED,  # Fixed joint
            jointAxis=[0, 0, 0],  # No axis for fixed joint
            parentFramePosition=position,  # Position relative to the parent (rectangle)
            childFramePosition=[0, 0, 0],  # Position relative to the wheel (origin for fixed joint)
            parentFrameOrientation=[0, 0, 0, 1],  # No rotation for the parent frame
            childFrameOrientation=[0, 0, 0, 1]  # No rotation for the wheel frame
        )

        return wheel_id

    def create_robot_variant_a(self):
        # Implement variant "A" if needed (this part is unchanged from the original)
        pass

    def update(self):
        pass  # Apply forces and update logic

    def get_sensor_data(self):
        data = {}
        for body in self.bodies:
            pos, orn = p.getBasePositionAndOrientation(body)
            lin_vel, ang_vel = p.getBaseVelocity(body)
            data[body] = {
                "position": pos,
                "velocity": lin_vel,
                "orientation": orn,
            }
        return data

    def get_forces(self):
        return self.forces  # Return current forces applied on bodies

    def get_bodies(self):
        return self.bodies


def load_model(physics_client, variant="default"):
    return Model(physics_client, variant)
