import numpy         as np
import visualisation as rob_vis

from model         import Rod, RodState, Cable, TensegrityRobot
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

# Visualise the initial state of the robot
rob_vis.plot(robot)

