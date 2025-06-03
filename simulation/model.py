import pybullet as p
import numpy as np
import os
import math
from math import pi
from .basescenario import RobotScenario
from .pendulum import PendulumScenario
from .four_wheel_pivot_straight import RobotFourWheelPivotStraight
from .four_wheel_pivot_curve import RobotFourWheelPivotCurve
from .three_wheel_pivot_straight import RobotThreeWheelPivotStraight
from .three_wheel_pivot_curve import RobotThreeWheelPivotCurve

# Top-level Model that wraps scenario
class Model:
    scenario_classes = {
        "default": PendulumScenario,
        "PENDULUM": PendulumScenario,
        "RobotFourWheelPivotStraight": RobotFourWheelPivotStraight,
        "RobotFourWheelPivotCurve": RobotFourWheelPivotCurve,
        "RobotThreeWheelPivotStraight": RobotThreeWheelPivotStraight,
        "RobotThreeWheelPivotCurve": RobotThreeWheelPivotCurve,
    }

    def __init__(self, physics_client, time_step, variant="default"):
        if variant not in self.scenario_classes:
            raise ValueError(f"Unknown variant: {variant}")
        self.time_step = time_step
        self.scenario = self.scenario_classes[variant](physics_client, self.time_step)
        self.scenario.load()

    def update(self):
        self.scenario.update()

    def get_sensor_data(self):
        return self.scenario.get_sensor_data()

    def get_list_of_segment(self):
        return self.scenario.get_list_of_segment()

    def print_joint_mapping(self):
        print("Joint Name → Index mapping:")
        for name, idx in self.scenario.joint_name_to_index.items():
            print(f"  {name} → {idx}")

    def print_link_mapping(self):
        print("Link Name → Index mapping:")
        for name, idx in self.scenario.link_name_to_index.items():
            print(f"  {name} → {idx}")

# Loader function
def load_model(physics_client, variant="default", time_step=1/120):
    return Model(physics_client, time_step, variant)
