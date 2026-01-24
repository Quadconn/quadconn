import sys
import pinocchio as pin
from pinocchio.visualize import GepettoVisualizer

from robot import Robot

class RobotVisualizer:

    def __init__(self, robot: Robot):
        model, collision_model, visual_model = pin.buildModelsFromUrdf(
                robot.get_urdf(), None, None 
        )
        self.visualizer = GepettoVisualizer(model, collision_model, visual_model)

        # Initialize the viewer.
        try:
            self.visualizer.initViewer()
        except ImportError as err:
            print(
                    "Error while initializing the viewer. "
                    "It seems you should install gepetto-viewer"
                    )
            print(err)
            sys.exit(0)

        try:
            self.visualizer.loadViewerModel()
        except AttributeError as err:
            print(
                    "Error while loading the viewer model. "
                    "It seems you should start gepetto-gui"
                    )
            print(err)
            sys.exit(0)

    def display(self, robot: Robot):
        self.visualizer.display(robot.get_configuration())
