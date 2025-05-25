from PyQt5.QtWidgets import QApplication
from simulation.simulator import Simulator
from simulation.model import load_model
from simulation.commands import CommandHandler
from visualization.plotter import Plotter

def main():
    app = QApplication([])  # Ensure QApplication is initialized first
    simulator = Simulator(time_step=1/240)
    model = load_model(simulator.physics_client, variant="B", time_step=1/240)

    plotter = Plotter(model.get_list_of_segment())

    simulator.register_plotter(plotter)
    commands = CommandHandler(simulator)
    simulator.register_command_handler(commands)

    simulator.run(model, record=False, plot=False)
    app.exec_()  # Keep PyQt event loop running

if __name__ == "__main__":
    main()
