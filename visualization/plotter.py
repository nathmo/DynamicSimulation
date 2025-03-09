import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from datetime import datetime
import threading
import time


class Plotter(QMainWindow):
    def __init__(self, bodies):
        super().__init__()
        self.setWindowTitle("Simulation Plots")
        self.bodies = bodies
        self.data = {body: {"time": [], "position": [], "velocity": [], "acceleration": [], "force": []} for body in
                     bodies}

        self.fig, self.axes = plt.subplots(len(bodies), 4, figsize=(12, 3 * len(bodies)))
        self.canvas = FigureCanvas(self.fig)
        self.lines = {}

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        central_widget.setLayout(layout)

        for i, body in enumerate(bodies):
            for j, metric in enumerate(["position", "velocity", "acceleration", "force"]):
                self.lines[(body, metric)], = self.axes[i, j].plot([], [], label=metric)
                self.axes[i, j].set_title(f"Body {body} - {metric}")
                self.axes[i, j].legend()

        self.show()

        # Get the initial minute when the program starts
        self.start_time = datetime.now()
        self.start_minute = self.start_time.minute

        # Start background thread to save plot every 2 seconds
        self.stop_thread = False
        self.save_thread = threading.Thread(target=self.save_plot_periodically)
        self.save_thread.start()

    def update(self, sensor_data, forces, time_step):
        for i, (body, metrics) in enumerate(self.data.items()):
            self.data[body]["time"].append(len(self.data[body]["time"]) * time_step)
            self.data[body]["position"].append(np.linalg.norm(sensor_data[body]["position"]))
            self.data[body]["velocity"].append(np.linalg.norm(sensor_data[body]["velocity"]))

            if len(self.data[body]["velocity"]) > 1:
                acc = (self.data[body]["velocity"][-1] - self.data[body]["velocity"][-2]) / time_step
                self.data[body]["acceleration"].append(acc)
            else:
                self.data[body]["acceleration"].append(0)

            self.data[body]["force"].append(np.linalg.norm(forces.get(body, [0, 0, 0])))

            for j, metric in enumerate(["position", "velocity", "acceleration", "force"]):
                self.lines[(body, metric)].set_xdata(self.data[body]["time"])
                self.lines[(body, metric)].set_ydata(self.data[body][metric])
                self.axes[i, j].relim()  # Ensure proper axes scaling
                self.axes[i, j].autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def closeEvent(self, event):
        # Stop the save thread when closing the window
        self.stop_thread = True
        self.save_thread.join()  # Wait for the thread to finish
        event.accept()

    def save_plot_periodically(self):
        last_minute = None
        while not self.stop_thread:
            # Get current time, but use the start time minute
            current_time = datetime.now()

            # If it's still the same minute as when the program started, save the plot
            if current_time.minute == self.start_minute:
                self.save_plots(current_time)
            else:
                # Wait for the next cycle if the minute changed (no saving during minute change)
                time.sleep(2)
                continue

            # Sleep for 2 seconds before checking again
            time.sleep(2)

    def save_plots(self, current_time):
        # Save plot every 2 seconds (overwrite previous image)
        os.makedirs("output_graph", exist_ok=True)
        timestamp = self.start_time.strftime("%Y_%m_%d_%H_%M")  # Use the minute from start time

        # Save the plot with a fixed filename to overwrite the previous one
        filename = f"output_graph/simulation_{timestamp}.png"
        self.fig.savefig(filename)
        print(f"Saved plot: {filename}")
