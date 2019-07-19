import numpy as np

from model         import Rod, RodState, Cable, TensegrityRobot
from visualisation import TensegrityRobotVisualiser
from copy          import deepcopy

# Let's construct a robot of two rods and one cable
rod1  = Rod(mass=1, inertia=np.eye(1), length=5, state=RodState())
rod2  = Rod(mass=1, inertia=np.eye(1), length=5, state=RodState(r=rod1.get_state().r + np.array([0, 0, 10])))
cab1  = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod2.get_endpoint_a(), stiffness=10, unstretched_length=0.1)
cab2  = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod2.get_endpoint_b(), stiffness=10, unstretched_length=0.1)

robot = TensegrityRobot()
robot.add_rods([rod1, rod2])
robot.add_cables([cab1, cab2])
print("First rod cables vectors")
print(rod1.get_cables_vectors())
print("Second rod cables vectors")
print(rod2.get_cables_vectors())

visualiser = TensegrityRobotVisualiser()
# Visualise the initial state of the robot
visualiser.plot_cur_state(robot)

# Collect data to visualise the historical data
n           = 0
hist_states = []
while n < 1000:
    # Update the robot
    robot.update(dt=0.001)

    # Collect updated data
    rod_states = [deepcopy(rod.get_state()) for rod in robot.get_rods()]
    hist_states.append(rod_states)

    n += 1

visualiser.animate_historical_states(robot, states=hist_states, interval=0.001)

