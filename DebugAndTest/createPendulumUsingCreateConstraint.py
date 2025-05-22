import pybullet as p
import pybullet_data
import time

# Connect to GUI
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf")

# Base block
base_half_extents = [0.1, 0.1, 0.1]
base_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=base_half_extents),
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=base_half_extents, rgbaColor=[1, 0, 0, 1]),
    basePosition=[0, 0, 2.0]
)

# Pendulum arm
arm_half_length = 0.5
arm_half_extents = [0.05, 0.05, arm_half_length]
arm_id = p.createMultiBody(
    baseMass=1.0,
    baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=arm_half_extents),
    baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=arm_half_extents, rgbaColor=[0, 0, 1, 1]),
    basePosition=[0, 0, 1.4]
)

# Disable collisions between parts
p.setCollisionFilterPair(base_id, arm_id, -1, -1, 0)

# Add revolute constraint
p.createConstraint(
    parentBodyUniqueId=base_id,
    parentLinkIndex=-1,
    childBodyUniqueId=arm_id,
    childLinkIndex=-1,
    jointType=p.JOINT_POINT2POINT, # JOINT_REVOLUTE   JOINT_POINT2POINT
    jointAxis=[1, 0, 0],
    parentFramePosition=[0, 0, -base_half_extents[2]],
    childFramePosition=[0, 0, 0.5]
)

# Add some swing
#p.resetBaseVelocity(arm_id, angularVelocity=[1.0, 0, 0])

# Simulate
while True:
    p.stepSimulation()
    time.sleep(1./240.)

# Conclusion, JOINT_REVOLUTE DO NOT WORK with create constraint
# This script will crash