import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# --- Simulation Parameters ---
SIMULATION_STEP = 1.0 / 240.0  # Simulation time step
WHEEL_RADIUS = 0.3  # meters
WHEEL_MASS = 5  # kg
TERRAIN_SIZE = 10  # Size of the terrain grid (in meters)
TERRAIN_RESOLUTION = 0.2  # Distance between grid points (in meters)
HEIGHT_SCALE = 0.2  # Scale of height variations
CAMERA_MOVE_SPEED = 0.2  # Speed of camera movement

# --- Initialize PyBullet ---
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, -9.81)
p.setTimeStep(SIMULATION_STEP)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Simulation control variables
simulation_running = True

def toggle_simulation():
    global simulation_running
    simulation_running = not simulation_running

# --- Generate Complex Terrain ---
def generate_heightfield(size, resolution, height_scale):
    grid_points = int(size / resolution)
    x = np.linspace(-size / 2, size / 2, grid_points)
    y = np.linspace(-size / 2, size / 2, grid_points)
    X, Y = np.meshgrid(x, y)

    # Example terrain: combination of sine waves and random noise
    height = 0.05 * np.sin(0.5 * X) * np.cos(0.5 * Y) + 0.01 * np.random.uniform(-1, 1, size=(grid_points, grid_points))+0.2*X
    return height.flatten(), grid_points

height_data, terrain_dim = generate_heightfield(TERRAIN_SIZE, TERRAIN_RESOLUTION, HEIGHT_SCALE)

terrain_shape = p.createCollisionShape(
    shapeType=p.GEOM_HEIGHTFIELD,
    meshScale=[TERRAIN_SIZE / terrain_dim, TERRAIN_SIZE / terrain_dim, 1],
    heightfieldTextureScaling=terrain_dim / 2,
    heightfieldData=height_data.tolist(),
    numHeightfieldRows=terrain_dim,
    numHeightfieldColumns=terrain_dim
)

terrain = p.createMultiBody(0, terrain_shape)
p.changeDynamics(terrain, -1, lateralFriction=1.0)

# Apply a texture to the terrain (optional)
texture_id = p.loadTexture("dirt.png")
p.changeVisualShape(terrain, -1, textureUniqueId=texture_id)

# --- Create Wheel ---
def create_wheel(position):
    wheel_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=WHEEL_RADIUS, height=0.05)
    wheel_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=WHEEL_RADIUS, length=0.05, rgbaColor=[0, 0, 0, 1])
    wheel = p.createMultiBody(
        baseMass=WHEEL_MASS,
        baseCollisionShapeIndex=wheel_collision,
        baseVisualShapeIndex=wheel_visual,
        basePosition=position,
        baseOrientation=p.getQuaternionFromEuler([math.pi / 2, 0, 0])  # Align cylinder along Y-axis
    )
    p.changeDynamics(wheel, -1, lateralFriction=1.2, rollingFriction=0.02, spinningFriction=0.01)
    return wheel

# --- Initialize Wheel ---
def reset_simulation():
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(SIMULATION_STEP)

    # Recreate terrain
    height_data, terrain_dim = generate_heightfield(TERRAIN_SIZE, TERRAIN_RESOLUTION, HEIGHT_SCALE)
    terrain_shape = p.createCollisionShape(
        shapeType=p.GEOM_HEIGHTFIELD,
        meshScale=[TERRAIN_SIZE / terrain_dim, TERRAIN_SIZE / terrain_dim, 1],
        heightfieldTextureScaling=terrain_dim / 2,
        heightfieldData=height_data.tolist(),
        numHeightfieldRows=terrain_dim,
        numHeightfieldColumns=terrain_dim
    )
    terrain = p.createMultiBody(0, terrain_shape)
    p.changeDynamics(terrain, -1, lateralFriction=1.0)
    p.changeVisualShape(terrain, -1, textureUniqueId=texture_id)

    # Recreate wheel
    return create_wheel([0, 0, WHEEL_RADIUS + 0.1])

wheel = reset_simulation()

# --- Camera Control ---
camera_target = [0, 0, 0]
camera_distance = 5
camera_yaw = 50
camera_pitch = -35
camera_position = [0, 0, 2]  # Initial camera position

p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target)

print("Controls:")
print("- Arrow keys: Move camera along X/Y axes")
print("- Page Up/Page Down: Move camera along Z axis")
print("- Space: toggle simulation time freeze")
print("- R: Reset simulation")
print("- ESC: Exit simulation")

# --- Run Simulation ---
while True:
    keys = p.getKeyboardEvents()

    # Camera movement
    if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
        camera_position[0] -= CAMERA_MOVE_SPEED
    if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
        camera_position[0] += CAMERA_MOVE_SPEED
    if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
        camera_position[1] += CAMERA_MOVE_SPEED
    if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
        camera_position[1] -= CAMERA_MOVE_SPEED
    if ord('9') in keys and keys[ord('9')] & p.KEY_IS_DOWN:  # Page Up (9 key as substitute)
        camera_position[2] += CAMERA_MOVE_SPEED
    if ord('3') in keys and keys[ord('3')] & p.KEY_IS_DOWN:  # Page Down (3 key as substitute)
        camera_position[2] -= CAMERA_MOVE_SPEED
    if p.B3G_SPACE in keys and keys[p.B3G_SPACE] & p.KEY_WAS_TRIGGERED:
        toggle_simulation()

    # Reset simulation with 'R'
    if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
        wheel = reset_simulation()
        camera_position = [0, 0, 2]

    # Exit with 'Q'
    if ord('q') in keys and keys[ord('q')] & p.KEY_WAS_TRIGGERED:
        print("Simulation terminated.")
        break
    current_view = p.getDebugVisualizerCamera()
    current_distance = current_view[10]  # Extract camera distance
    current_yaw = current_view[8]  # Extract camera yaw
    current_pitch = current_view[9]  # Extract camera pitch

    # Update camera view
    p.resetDebugVisualizerCamera(current_distance, current_yaw, current_pitch, camera_position)
    # Toggle simulation when space bar is pressed


    if simulation_running:
        p.stepSimulation()

    time.sleep(SIMULATION_STEP)

p.disconnect()
