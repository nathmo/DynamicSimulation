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

        # bodies should be a list of link indices, e.g. [-1, 0, 1, ..., 12]
        self.bodies = bodies
        self.n_bodies = len(bodies)

        # Map link_index to row index in plot grid
        self.link_to_row = {link: i for i, link in enumerate(self.bodies)}

        # Initialize data dict for all bodies
        self.data = {
            link: {"time": [], "position": [], "velocity": [], "acceleration": [], "force": []}
            for link in self.bodies
        }
        # Create subplots once for all bodies and 4 metrics
        self.fig, self.axes = plt.subplots(self.n_bodies, 4, figsize=(12, 3 * self.n_bodies), squeeze=False)
        self.canvas = FigureCanvas(self.fig)

        # Initialize line plots for each link and metric
        self.lines = {}
        for i, link in enumerate(self.bodies):
            for j, metric in enumerate(["position", "velocity", "acceleration", "force"]):
                line, = self.axes[i, j].plot([], [], label=metric)
                self.axes[i, j].set_title(f"Body {link} - {metric}")
                self.axes[i, j].legend()
                self.lines[(link, metric)] = line

        # Qt layout setup
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        central_widget.setLayout(layout)

        self.show()
        self._last_refresh_time = 0
        # Start time for naming saved plots
        self.start_time = datetime.now()
        self.start_minute = self.start_time.minute

    def update_sensor_data(self, sensor_data, time_step):
        for link_index, metrics in sensor_data.items():
            if link_index not in self.data:
                print("link_index not found in previous data : " + str(link_index))
                continue
            self.data[link_index]["time"].append(len(self.data[link_index]["time"]) * time_step)
            self.data[link_index]["position"].append(np.linalg.norm(metrics["position"]))
            self.data[link_index]["velocity"].append(np.linalg.norm(metrics["velocity"]))
            self.data[link_index]["acceleration"].append(np.linalg.norm(metrics["acceleration"]))
            self.data[link_index]["force"].append(np.linalg.norm(metrics["force"]))
        now = time.time()
        if now - self._last_refresh_time > 5.0:  # only update the plot once every 5 sec
            self.refresh_plot()
            self.save_plots()
            self._last_refresh_time = now


    def refresh_plot(self):
        for link_index in self.data:
            row = self.link_to_row[link_index]
            for j, metric in enumerate(["position", "velocity", "acceleration", "force"]):
                line = self.lines[(link_index, metric)]
                line.set_xdata(self.data[link_index]["time"])
                line.set_ydata(self.data[link_index][metric])
                ax = self.axes[row, j]
                ax.relim()
                ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def save_plots(self):
        os.makedirs("output_graph", exist_ok=True)
        timestamp = self.start_time.strftime("%Y_%m_%d_%H_%M")
        filename = f"output_graph/simulation_{timestamp}.png"
        self.fig.savefig(filename)
        print(f"Saved plot: {filename}")
