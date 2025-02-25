import pybullet as p

class CommandHandler:
    def __init__(self, simulator):
        self.simulator = simulator
        self.freeze = False

    def process_input(self):
        keys = p.getKeyboardEvents()
        if ord('f') in keys and keys[ord('f')] & p.KEY_WAS_TRIGGERED:
            self.freeze = not self.freeze
            print("Simulation frozen" if self.freeze else "Simulation resumed")

        if not self.freeze:
            self.handle_movement(keys)

    def handle_movement(self, keys):
        if ord('w') in keys:
            p.applyExternalForce(self.simulator.robot_id, -1, [10, 0, 0], [0, 0, 0], p.WORLD_FRAME)
