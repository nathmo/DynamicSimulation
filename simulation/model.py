import pybullet as p

def load_model(physics_client):
    plane = p.loadURDF("output_mesh/plane.urdf", useFixedBase=True)
    robot = p.loadURDF("robot.urdf", [0, 0, 0])
    return Model(plane, robot)

class Model:
    def __init__(self, plane_id, robot_id):
        self.plane_id = plane_id
        self.robot_id = robot_id

    def update(self):
        # Update model states (e.g., apply forces)
        pass

    def get_sensor_data(self):
        position, orientation = p.getBasePositionAndOrientation(self.robot_id)
        linear_vel, angular_vel = p.getBaseVelocity(self.robot_id)
        return {"position": position, "velocity": linear_vel, "orientation": orientation}
