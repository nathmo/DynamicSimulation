import pybullet as p
import pybullet_data
import math

class CommandHandler:
    def __init__(self, simulator):
        self.simulator = simulator
        self.freeze = False
        self.camera_distance = 3.0
        self.camera_yaw = 50.0
        self.camera_pitch = -35.0
        self.camera_target = [0, 0, 0]

    def process_input(self):
        keys = p.getKeyboardEvents()

        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
            p.resetSimulation()
            print("Simulation reset")

        if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
            self.freeze = not self.freeze
            print("Simulation paused" if self.freeze else "Simulation resumed")

        if not self.freeze:
            self.handle_movement(keys)

        self.handle_camera(keys)

    def handle_movement(self, keys):
        pass

    def handle_camera(self, keys):
        # Pitch and yaw control (WASD)
        if ord('z') in keys and keys[ord('z')] & p.KEY_IS_DOWN:
            self.camera_pitch = max(self.camera_pitch - 1, -89)
        if ord('h') in keys and keys[ord('h')] & p.KEY_IS_DOWN:
            self.camera_pitch = min(self.camera_pitch + 1, 89)
        if ord('g') in keys and keys[ord('g')] & p.KEY_IS_DOWN:
            self.camera_yaw -= 1
        if ord('j') in keys and keys[ord('j')] & p.KEY_IS_DOWN:
            self.camera_yaw += 1

        # Zoom control (Q and E)
        if ord('t') in keys and keys[ord('t')] & p.KEY_IS_DOWN:
            self.camera_distance = max(self.camera_distance - 0.1, 0.1)
        if ord('u') in keys and keys[ord('u')] & p.KEY_IS_DOWN:
            self.camera_distance += 0.1

        # Camera target translation relative to camera orientation
        move_step = 0.1
        yaw_rad = math.radians(self.camera_yaw)

        forward = [math.cos(yaw_rad), math.sin(yaw_rad), 0]
        right = [-math.sin(yaw_rad), math.cos(yaw_rad), 0]

        if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
            self.camera_target[0] -= forward[0] * move_step
            self.camera_target[1] -= forward[1] * move_step
        if p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
            self.camera_target[0] += forward[0] * move_step
            self.camera_target[1] += forward[1] * move_step
        if p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN:
            self.camera_target[0] += right[0] * move_step
            self.camera_target[1] += right[1] * move_step
        if p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN:
            self.camera_target[0] -= right[0] * move_step
            self.camera_target[1] -= right[1] * move_step

        # Update the camera view
        p.resetDebugVisualizerCamera(self.camera_distance, self.camera_yaw, self.camera_pitch, self.camera_target)
