# NOTE: this example needs gepetto-gui to be installed
# usage: launch gepetto-gui and then run this test
 
import sys
from pathlib import Path
 
import pinocchio as pin
from pinocchio.visualize import GepettoVisualizer
 

urdf_model_path = Path(__file__).parent.parent / "urdf/leg3dof.urdf"
 
model, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, None, None 
)

viz = GepettoVisualizer(model, collision_model, visual_model)
 
# Initialize the viewer.
try:
    viz.initViewer()
except ImportError as err:
    print(
        "Error while initializing the viewer. "
        "It seems you should install gepetto-viewer"
    )
    print(err)
    sys.exit(0)
 
try:
    viz.loadViewerModel("pinocchio")
except AttributeError as err:
    print(
        "Error while loading the viewer model. "
        "It seems you should start gepetto-viewer"
    )
    print(err)
    sys.exit(0)
 
# Display a robot configuration.
q0 = pin.neutral(model)
viz.display(q0)
