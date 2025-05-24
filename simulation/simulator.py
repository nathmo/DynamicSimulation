import pybullet as p
import pybullet_data
import time
import os
from datetime import datetime


class Simulator:
    def __init__(self, time_step=1 / 240):
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self.time_step = time_step
        self.command_handler = None
        self.plotter = None
        self.video_log_id = None

        # Ensure output_video directory exists
        self.video_dir = "output_video"
        os.makedirs(self.video_dir, exist_ok=True)

    def register_command_handler(self, handler):
        self.command_handler = handler

    def register_plotter(self, plotter):
        self.plotter = plotter  # QApplication should already be instantiated in main.py

    def get_next_filename(self):
        """Generate the next available video filename with timestamp."""
        timestamp = datetime.now().strftime("%Y_%m_%d_%H_%M")
        base_name = os.path.join(self.video_dir, f"simulation_{timestamp}")
        index = 1
        while os.path.exists(f"{base_name}_{index}.mp4"):
            index += 1
        return f"{base_name}_{index}.mp4"

    def start_recording(self):
        """Start recording the simulation to an MP4 file."""
        filename = self.get_next_filename()
        self.video_log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, filename)
        print(f"Recording simulation to {filename}")

    def stop_recording(self):
        """Stop recording the simulation."""
        if self.video_log_id is not None:
            p.stopStateLogging(self.video_log_id)
            print("Stopped recording")
            self.video_log_id = None

    def run(self, model, record=True):
        if record:
            self.start_recording()

        try:
            while True:
                if self.command_handler:
                    self.command_handler.process_input()

                model.update()
                p.stepSimulation()

                if self.plotter:
                    self.plotter.update(model.get_sensor_data(), self.time_step)

                time.sleep(self.time_step)
        except KeyboardInterrupt:
            pass  # Graceful exit on manual interruption

        if record:
            self.stop_recording()

        if self.plotter:
            self.plotter.save_plots()
