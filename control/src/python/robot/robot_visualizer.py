import pinocchio as pin
from pinocchio.visualize import GepettoVisualizer

from robot import Robot

class RobotVisualizer:

    def __init__(self, robot: Robot, node_name: str):
        model, collision_model, visual_model = pin.buildModelsFromUrdf(
                robot.get_urdf(), None, None 
        )
        # TODO: Add runtime errors if visualizer fails to open
        self.visualizer = GepettoVisualizer(model, collision_model, visual_model)

        self.visualizer.initViewer()
        self.visualizer.loadViewerModel(node_name)

    def display(self, robot: Robot):
        self.visualizer.display(robot.get_configuration())
