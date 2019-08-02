import numpy         as np
from math import cos, sin, pi
import visualisation as rob_vis

from model                   import Rod, RodState, Cable, TensegrityRobot, run_simulation
from copy                    import deepcopy
from scipy.spatial.transform import Rotation

LENGTH = 0.3
OFFSET = LENGTH/8.0
UNSTRETCHED_LENGTH_v = 0.2
UNSTRETCHED_LENGTH_h = 0.1
STIFFNESS = 10
VISCOSITY = 1

q0=Rotation.from_euler("xyz", [0, 90, 0], degrees=True)

q1= Rotation.from_euler("xyz", [0, -30, 0], degrees=True) * q0
q2= Rotation.from_euler("xyz", [0, 0, 120], degrees=True) * q1
q3= Rotation.from_euler("xyz", [0, 0, 120], degrees=True) * q2

rod1  = Rod(mass=1, inertia=np.eye(3), length=LENGTH, state=RodState(r=np.array([ OFFSET*cos(0*pi/3+pi/3),  OFFSET*sin(0*pi/3+pi/3), 0]), q=q1))
rod2  = Rod(mass=1, inertia=np.eye(3), length=LENGTH, state=RodState(r=np.array([ OFFSET*cos(2*pi/3+pi/3),  OFFSET*sin(2*pi/3+pi/3), 0]), q=q2))
rod3  = Rod(mass=1, inertia=np.eye(3), length=LENGTH, state=RodState(r=np.array([ OFFSET*cos(4*pi/3+pi/3),  OFFSET*sin(4*pi/3+pi/3), 0]), q=q3))

# Upper cables
cab1  = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod3.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h, viscosity=VISCOSITY)
cab2  = Cable(end_point1=rod3.get_endpoint_a(), end_point2=rod2.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h, viscosity=VISCOSITY)
cab3  = Cable(end_point1=rod2.get_endpoint_a(), end_point2=rod1.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h, viscosity=VISCOSITY)

# Lower cables
cab4  = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod2.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h, viscosity=VISCOSITY)
cab5  = Cable(end_point1=rod2.get_endpoint_b(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h, viscosity=VISCOSITY)
cab6  = Cable(end_point1=rod3.get_endpoint_b(), end_point2=rod1.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h, viscosity=VISCOSITY)

# Side cables
cab7  = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod2.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_v, viscosity=VISCOSITY)
cab8  = Cable(end_point1=rod2.get_endpoint_a(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_v, viscosity=VISCOSITY)
cab9  = Cable(end_point1=rod3.get_endpoint_a(), end_point2=rod1.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_v, viscosity=VISCOSITY)

robot = TensegrityRobot()
robot.add_rods([rod1, rod2, rod3])
robot.add_cables([cab1, cab2, cab3, cab4, cab5, cab6, cab7, cab8, cab9])



rob_vis.plot_cur_state(robot)
hist_states = run_simulation(robot, time=3, dt=0.005)
rob_vis.plot_cur_state(robot)

rob_vis.plot_com_graphs(hist_states)
#rob_vis.animate_historical_states(robot=robot, states=hist_states, interval=0.01)

