import pybullet as p
import numpy as np


class Segment:
    """Represents a segment (rigid body) in the kinematic chain."""

    def __init__(self, physics_client, shape, mass, position, orientation=[0, 0, 0, 1]):
        self.physics_client = physics_client
        self.shape = shape
        self.mass = mass
        self.position = position
        self.orientation = orientation
        self.body_id = self.create_body()

    def create_body(self):
        """Create and return a rigid body in PyBullet."""
        collision_shape = p.createCollisionShape(self.shape[0], halfExtents=self.shape[1])
        body_id = p.createMultiBody(
            baseMass=self.mass,
            baseCollisionShapeIndex=collision_shape,
            basePosition=self.position,
            baseOrientation=self.orientation
        )
        return body_id


class Joint:
    """Represents a joint connecting two segments."""

    def __init__(self, physics_client, parent, child, joint_type, joint_axis, parent_offset, child_offset):
        self.physics_client = physics_client
        self.parent = parent
        self.child = child
        self.joint_type = joint_type
        self.joint_axis = joint_axis
        self.parent_offset = parent_offset
        self.child_offset = child_offset
        self.create_joint()
        self.visualize_joint()

    def create_joint(self):
        """Create a joint between two segments."""
        p.createConstraint(
            parentBodyUniqueId=self.parent.body_id,
            parentLinkIndex=-1,
            childBodyUniqueId=self.child.body_id,
            childLinkIndex=-1,
            jointType=self.joint_type,
            jointAxis=self.joint_axis,
            parentFramePosition=self.parent_offset,
            childFramePosition=self.child_offset,
            parentFrameOrientation=[0, 0, 0, 1],
            childFrameOrientation=[0, 0, 0, 1]
        )

    def visualize_joint(self):
        """Display a red arrow showing the joint axis and attachment point."""
        start_pos = np.array(self.parent.position) + np.array(self.parent_offset)
        end_pos = start_pos + 0.2 * np.array(self.joint_axis)  # Scale arrow
        p.addUserDebugLine(start_pos, end_pos, [1, 0, 0], 5.0)  # Red arrow


class Model:
    """Manages the full kinematic chain, including segments and joints."""

    def __init__(self, physics_client, variant="default"):
        self.physics_client = physics_client
        self.bodies = []
        self.segments = []
        self.joints = []
        self.forces = {body: [0, 0, 0] for body in self.bodies}
        self.variant = variant
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
            self.create_robot_default()
        elif self.variant == "PENDULUM":
            self.create_robot_variant_pendulum()
        elif self.variant == "A":
            self.create_robot_variant_a()
        else:
            raise ValueError(f"Unknown robot variant: {self.variant}")

    def load_surface(self):
        """Load the ground surface."""
        plane_id = p.loadURDF("urdf/plane.urdf", basePosition=[0, 0, -0.05], useFixedBase=True)
        self.bodies.append(plane_id)

    def add_segment(self, shape, mass, position, orientation=[0, 0, 0, 1]):
        """Add a new segment to the model."""
        segment = Segment(self.physics_client, shape, mass, position, orientation)
        self.segments.append(segment)
        self.bodies.append(segment.body_id)
        return segment

    def add_joint(self, parent, child, joint_type, joint_axis, parent_offset, child_offset):
        """Add a joint connecting two segments."""
        joint = Joint(self.physics_client, parent, child, joint_type, joint_axis, parent_offset, child_offset)
        self.joints.append(joint)

    def create_robot_default(self):
        """Define the default kinematic chain."""
        base = self.add_segment((p.GEOM_BOX, [1.0, 0.3, 0.05]), mass=10.0, position=[0, 0, 0.05])

        wheel_positions = [
            [-1, 0.3, 0],  # A
            [1, 0.3, 0],  # B
            [-1, -0.3, 0],  # C
            [1, -0.3, 0]  # D
        ]

        wheels = []
        for pos in wheel_positions:
            wheel = self.add_segment((p.GEOM_CYLINDER, [0.35, 0.05]), mass=1.0, position=pos)
            wheels.append(wheel)

        # Attach wheels with fixed joints
        for wheel, pos in zip(wheels, wheel_positions):
            self.add_joint(base, wheel, p.JOINT_FIXED, [1, 0, 0], pos, [0, 0, 0])

    def create_robot_variant_pendulum(self):
        """Creates a simple swinging pendulum using createConstraint."""

        # Fixed base (acts as the pivot, no mass)
        base = self.add_segment((p.GEOM_BOX, [0.2, 0.2, 0.2]), mass=0, position=[0, 0, 2.0])

        # Pendulum arm (box vertically hanging, so we offset the center of mass)
        arm = self.add_segment((p.GEOM_BOX, [0.05, 0.05, 1.0]), mass=2.0, position=[0, 0, 0.8], orientation=[0,0,0])
        p.setCollisionFilterPair(base.body_id, arm.body_id, -1, -1, 0)  # disable collision between base and arm

        # Let the world stabilize before applying constraint
        p.stepSimulation()

        # Create a point2point constraint (acts like a ball joint)
        # Pivot is at the bottom of the base and top of the pendulum arm
        self.add_joint(base, arm, p.JOINT_REVOLUTE, [1, 0, 0], [0, 0, -0.2], [0, 0, 1])

    def create_robot_variant_a(self):
        """Alternative kinematic structure."""

        base = self.add_segment((p.GEOM_BOX, [1.0, 0.3, 0.05]), mass=10.0, position=[0, 0, 1.05])

        arm = self.add_segment((p.GEOM_BOX, [0.05, 0.1, 0.5]), mass=2.0, position=[0.5, 0, 2.15])

        self.add_joint(base, arm, p.JOINT_PRISMATIC, [0, 0, 1], [0, 0, 0], [0, 0, 0])

    def update(self):
        """Placeholder for physics update logic."""
        pass

    def get_sensor_data(self):
        """Get the state of all bodies."""
        data = {}
        for segment in self.segments:
            pos, orn = p.getBasePositionAndOrientation(segment.body_id)
            lin_vel, ang_vel = p.getBaseVelocity(segment.body_id)
            data[segment.body_id] = {
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
