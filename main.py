from simulation.simulator import Simulator
from simulation.model import load_model
from simulation.commands import CommandHandler
from visualization.plotter import Plotter

def main():
    simulator = Simulator(time_step=1/240)
    model = load_model(simulator.physics_client)
    plotter = Plotter()
    commands = CommandHandler(simulator)

    simulator.register_plotter(plotter)
    simulator.register_command_handler(commands)

    simulator.run(model)

if __name__ == "__main__":
    main()
