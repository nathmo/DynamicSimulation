import pybullet as p
import pybullet_data
import time

# Connect to GUI
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane for reference
p.loadURDF("plane.urdf")

# Base (fixed in space)
base_half_extents = [0.1, 0.1, 0.1]
base_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=base_half_extents)
base_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=base_half_extents, rgbaColor=[1, 0, 0, 1])

# Pendulum arm (pivot, first arm)
pivot_half_extents = [0.1, 0.1, 0.1]
pivot_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=pivot_half_extents)
pivot_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=pivot_half_extents, rgbaColor=[0, 0, 1, 1])

# Create second pendulum arm (attached to the first arm)
second_arm_half_extents = [0.1, 0.5, 0.1]
second_arm_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=second_arm_half_extents)
second_arm_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=second_arm_half_extents, rgbaColor=[0, 1, 0, 1])

# Create the first pendulum (attached to the base)
pendulum_id = p.createMultiBody(
    baseMass=0,  # Mass of the base (0 means it's fixed)
    baseCollisionShapeIndex=base_col,  # Collision shape index for the base
    baseVisualShapeIndex=base_vis,  # Visual shape index for the base
    basePosition=[0, 0, 2.0],  # Position of the base in world coordinates
    baseOrientation=[0, 0, 0, 1],  # The base orientation (no rotation)
    baseInertialFramePosition=[0, 0, 0],  # Center of mass of the base
    baseInertialFrameOrientation=[0, 0, 0, 1],  # Inertial orientation of the base

    linkMasses=[0.0, 10.0],  # Mass of the pivot and second arm
    linkCollisionShapeIndices=[pivot_col, second_arm_col],  # Collision shape index for the arm
    linkVisualShapeIndices=[pivot_vis, second_arm_vis],  # Visual shape index for the arm
    linkPositions=[[0.2, 0, 0], [0, 0.6, 0]],  # Position of the arm relative to the base
    linkOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],  # Orientation of the arm (no rotation)
    linkInertialFramePositions=[[0, 0, 0], [0, 0, 0]],  # COM of the arm relative to the arm's frame
    linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],  # Inertial frame orientation (default)
    linkParentIndices=[0, 1],  # The parent of the arm is the base (index 0)
    linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_FIXED],  # Both joints should be revolute
    linkJointAxis=[[1, 0, 0], [0, 0, 1]],  # The joint axis is along the X-axis of the parent (base)
)

num_joints = p.getNumJoints(pendulum_id)
for j in range(num_joints):
    p.changeDynamics(pendulum_id, j, linearDamping=0.000001, angularDamping=0.000001, jointDamping=0.000001)


# Simulate
while True:
    p.stepSimulation()
    time.sleep(1./240.)
