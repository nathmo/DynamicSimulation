import pybullet as p
import pybullet_data
import time

class Simulator:
    def __init__(self, time_step=1/240):
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.time_step = time_step
        self.command_handler = None
        self.plotter = None

    def register_command_handler(self, handler):
        self.command_handler = handler

    def register_plotter(self, plotter):
        self.plotter = plotter

    def run(self, model):
        while True:
            if self.command_handler:
                self.command_handler.process_input()

            model.update()
            p.stepSimulation()

            if self.plotter:
                self.plotter.update(model.get_sensor_data())

            time.sleep(self.time_step)
