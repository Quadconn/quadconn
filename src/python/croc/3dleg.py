import crocoddyl
import numpy as np
import pinocchio
import meshcat.geometry as mg
import pathlib
import matplotlib.pyplot as plt
from IPython.display import display

# provide path to urdf file for pinocchio
urdf_model_path = pathlib.Path(
    '/home/quadconn/quadconn/croc_/1leg.urdf')
robot = pinocchio.robot_wrapper.RobotWrapper.BuildFromURDF(
    str(urdf_model_path))
# prints model using pinocchio
print(robot.model)
# create the multibody state from the pinocchio model
state = crocoddyl.StateMultibody(robot.model)


class LegActuationModel(crocoddyl.ActuationModelAbstract):
    def __init__(self, state):
        nu = 3  # control dimension or #DOF
        crocoddyl.ActuationModelAbstract.__init__(self, state, nu=nu)

    def calc(self, data, x, u):
        assert len(data.tau) == 3  # tau exists in R^3
        data.tau[0] = u[0]
        data.tau[1] = u[1]
        data.tau[2] = u[2]
        # this is one to one mapping so each joint will receive an actuation signal

    def calcDiff(self, data, x, u):
        # specify the actuatuion jacobian, just to specify one-to-one mapping
        # saying hey joint 0 affect joint 0
        data.dtau_du[0, 0] = 1
        data.dtau_du[1, 1] = 1
        data.dtau_du[2, 2] = 1


# data.dtau_du exists in r^9 since r^3 times r^3
actuationModel = LegActuationModel(state)

dt = 1e-3  # Time Step (1 millisecond)
T = 1000  # Number of knots

# Cost Models
runningCostModel = crocoddyl.CostModelSum(state, nu=actuationModel.nu)
terminalCostModel = crocoddyl.CostModelSum(state, nu=actuationModel.nu)

# Add a cost for the configuration positions and velocities
xref = np.array([0, 0, 0, 0, 0, 0])
stateResidual = crocoddyl.ResidualModelState(
    state, xref=xref, nu=actuationModel.nu)
stateCostModel = crocoddyl.CostModelResidual(state, stateResidual)
# runningCostModel L(x,u,t) that measures immediate robot performance and penalizes behavior with weights
runningCostModel.addCost("state_cost", cost=stateCostModel, weight=1e-5 / dt)
# Terminal Cost Model penalizes optimal control problem based on evaluation of final state to make the problem solvable, estimates costs
terminalCostModel.addCost("state_cost", cost=stateCostModel, weight=1000)

# Add a cost on control
# defines a residual vector as r = u - uref
controlResidual = crocoddyl.ResidualModelControl(state, nu=actuationModel.nu)
bounds = crocoddyl.ActivationBounds(
    np.array([-1.0, -1.0, -1.0]), np.array([1.0, 1.0, 1.0]))
activation = crocoddyl.ActivationModelQuadraticBarrier(bounds)
controlCost = crocoddyl.CostModelResidual(
    state, activation=activation, residual=controlResidual)
runningCostModel.addCost("control cost", cost=controlCost, weight=1e-1 / dt)

# Create the action models for the state
runningModel = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuationModel, runningCostModel), dt)
terminalModel = crocoddyl.IntegratedActionModelEuler(
    crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuationModel, runningCostModel), 0.0)

# Have initiliazed the cost and action model, now define the control objective
# give initial states of joing positions and veolocities, then combine it into robot initial state
q0 = np.zeros(state.nq)
q0[0] = np.pi * 2
q0[1] = 0
q0[2] = 0
v0 = np.zeros(state.nv)
x0 = np.concatenate((q0, v0), axis=0)
problem = crocoddyl.ShootingProblem(x0, [runningModel] * T, terminalModel)

# test it
# 0.01 is the input singal to that joint in form or array, and do this up until T times
us = [0.01 * np.ones((3)) for _ in range(T)]
xs = problem.rollout(us)

# Handy to blat up the state and and control trajectories
crocoddyl.plotOCSolution(
    xs, us, show=False, figIndex=99, figTitle="Test Rollout")

# Put a grid on the plots
# This creates the figure
fig = plt.gcf()
axs = fig.axes
for ax in axs:
    ax.grid

# solve the problem using Feasability-driven DDP
solver = crocoddyl.SolverFDDP(problem)

# Diagnose the solver and solve
callbacks = []
callbacks.append(crocoddyl.CallbackLogger())
callbacks.append(crocoddyl.CallbackVerbose())
solver.setCallbacks(callbacks)
solver.solve([], [], 300, False, 1e-5)

# Display using meshcat
robot_display = crocoddyl.MeshcatDisplay(
    robot=robot, rate=-1, freq=1, cameraTF=False)
display(robot_display.robot.viewer.jupyter_cell())
robot_display.displayFromSolver(solver)

# Plot the solution and the DDP convergence
log = solver.getCallbacks()[0]
crocoddyl.plotOCSolution(xs=solver.xs, us=solver.us, figIndex=1,
                         show=False, figTitle="Solution")
fig = plt.gcf()
axs = fig.axes
for ax in axs:
    ax.grid(True)

crocoddyl.plotConvergence(
    log.costs, log.pregs, log.dregs, log.grads, log.stops, log.steps, figIndex=2, show=False)
fig = plt.gcf()
axs = fig.axes
for ax in axs:
    ax.grid(True)

plt.show
