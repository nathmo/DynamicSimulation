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
pivot_half_extents = [0.1, 0.1, 0.3]
pivot_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=pivot_half_extents)
pivot_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=pivot_half_extents, rgbaColor=[0, 0, 1, 1])

# Create second pendulum arm (attached to the first arm)
second_arm_half_extents = [0.1, 0.5, 0.1]
second_arm_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=second_arm_half_extents)
second_arm_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=second_arm_half_extents, rgbaColor=[0, 1, 0, 1])

# Create the pendulum
pendulum_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=base_col,
    baseVisualShapeIndex=base_vis,
    basePosition=[0, 0, 2.0],
    baseOrientation=[0, 0, 0, 1],
    baseInertialFramePosition=[0, 0, 0],
    baseInertialFrameOrientation=[0, 0, 0, 1],
    linkMasses=[1.0, 100.0],
    linkCollisionShapeIndices=[pivot_col, second_arm_col],
    linkVisualShapeIndices=[pivot_vis, second_arm_vis],
    linkPositions=[[0.2, 0, 0.2], [0, 0.6, 0]],
    linkOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
    linkInertialFramePositions=[[0, 0, 0], [0, 0.0, 0.0]],
    linkInertialFrameOrientations=[[0, 0, 0, 1], [0, 0, 0, 1]],
    linkParentIndices=[0, 1],
    linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_FIXED],
    linkJointAxis=[[1, 0, 0], [0, 0, 1]],
)

# Disable all default damping/friction/motor resistance
num_joints = p.getNumJoints(pendulum_id)
for i in range(-1, num_joints):  # -1 = base
    p.changeDynamics(pendulum_id, i,
                     lateralFriction=0.0,
                     spinningFriction=0.0,
                     rollingFriction=0.0,
                     linearDamping=0.0,
                     angularDamping=0.0)

for i in range(num_joints):
    info = p.getJointInfo(pendulum_id, i)
    joint_type = info[2]
    if joint_type == p.JOINT_REVOLUTE:
        # Disable default motor (acts like joint friction if left enabled)
        p.setJointMotorControl2(pendulum_id, i,
                                controlMode=p.VELOCITY_CONTROL,
                                force=0)

# Run sim
while True:
    p.stepSimulation()
    time.sleep(1./240.)
