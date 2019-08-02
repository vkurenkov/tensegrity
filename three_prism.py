import numpy         as np
from math import cos, pi
import visualisation as rob_vis

from model                   import Rod, RodState, Cable, TensegrityRobot, run_simulation
from copy                    import deepcopy
from scipy.spatial.transform import Rotation

LENGTH = 5.0
OFFSET = LENGTH/8.0
UNSTRETCHED_LENGTH = 0.1
STIFFNESS = 10
VISCOSITY = 1

L = 0.05

rod1  = Rod(mass=1, inertia=np.eye(3), length=LENGTH, state=RodState(r=np.array([ L,            0,           0]), q=Rotation.from_euler("xyz", [0, -85, 0], degrees=True)))
rod2  = Rod(mass=1, inertia=np.eye(3), length=LENGTH, state=RodState(r=np.array([ L*cos(pi/6),  L*cos(pi/3), 0]), q=Rotation.from_euler("xyz", [0, -85, 120], degrees=True)))
rod3  = Rod(mass=1, inertia=np.eye(3), length=LENGTH, state=RodState(r=np.array([-L*cos(pi/6), -L*cos(pi/3), 0]), q=Rotation.from_euler("xyz", [0, -85, 240], degrees=True)))

cab1  = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod2.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab2  = Cable(end_point1=rod2.get_endpoint_a(), end_point2=rod3.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab3  = Cable(end_point1=rod3.get_endpoint_a(), end_point2=rod1.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab4  = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod2.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab5  = Cable(end_point1=rod2.get_endpoint_b(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab6  = Cable(end_point1=rod3.get_endpoint_b(), end_point2=rod1.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab7  = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod2.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab8  = Cable(end_point1=rod2.get_endpoint_b(), end_point2=rod3.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab9  = Cable(end_point1=rod3.get_endpoint_b(), end_point2=rod1.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)



robot = TensegrityRobot()
robot.add_rods([rod1, rod2, rod3])
robot.add_cables([cab1, cab2, cab3, cab4, cab5, cab6, cab7, cab8, cab9])
rob_vis.plot_cur_state(robot)
hist_states = run_simulation(robot, time=5, dt=0.005)

rob_vis.plot_cur_state(robot)

#rob_vis.animate_historical_states(robot=robot, states=hist_states, interval=0.01)
