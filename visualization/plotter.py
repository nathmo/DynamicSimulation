import matplotlib.pyplot as plt
import threading

class Plotter:
    def __init__(self):
        self.data = {"time": [], "position": []}
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-')
        threading.Thread(target=self.show_plot, daemon=True).start()

    def update(self, sensor_data):
        self.data["time"].append(len(self.data["time"]))
        self.data["position"].append(sensor_data["position"][0])

        self.line.set_xdata(self.data["time"])
        self.line.set_ydata(self.data["position"])
        self.ax.relim()
        self.ax.autoscale_view()

    def show_plot(self):
        plt.ion()
        plt.show()
