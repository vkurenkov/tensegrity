import numpy         as np
from math import cos, sin, pi
import visualisation as rob_vis

from model                   import Rod, RodState, Cable, TensegrityRobot, run_simulation
from copy                    import deepcopy
from scipy.spatial.transform import Rotation

LENGTH = 0.3
OFFSET = LENGTH/8.0
UNSTRETCHED_LENGTH_v = 0.2
UNSTRETCHED_LENGTH_h1 = 0.03
UNSTRETCHED_LENGTH_h2 = 0.15
STIFFNESS = 10
VISCOSITY = 1

q0=Rotation.from_euler("xyz", [0, 90, 0], degrees=True)

q1= Rotation.from_euler("xyz", [0, -30, 0], degrees=True) * q0
q2= Rotation.from_euler("xyz", [0, 0, 90], degrees=True) * q1
q3= Rotation.from_euler("xyz", [0, 0, 90], degrees=True) * q2
q4= Rotation.from_euler("xyz", [0, 0, 90], degrees=True) * q3

rod1  = Rod(mass=1, inertia=np.eye(3), length=LENGTH, state=RodState(r=np.array([ OFFSET*cos(0*pi/2+pi/4),  OFFSET*sin(0*pi/2+pi/4), 0]), q=q1))
rod2  = Rod(mass=1, inertia=np.eye(3), length=LENGTH, state=RodState(r=np.array([ OFFSET*cos(1*pi/2+pi/4),  OFFSET*sin(1*pi/2+pi/4), 0]), q=q2))
rod3  = Rod(mass=1, inertia=np.eye(3), length=LENGTH, state=RodState(r=np.array([ OFFSET*cos(2*pi/2+pi/4),  OFFSET*sin(2*pi/2+pi/4), 0]), q=q3))
rod4  = Rod(mass=1, inertia=np.eye(3), length=LENGTH, state=RodState(r=np.array([ OFFSET*cos(3*pi/2+pi/4),  OFFSET*sin(3*pi/2+pi/4), 0]), q=q4))

# Upper cables
cab1  = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod4.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h1, viscosity=VISCOSITY)
cab2  = Cable(end_point1=rod2.get_endpoint_a(), end_point2=rod1.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h1, viscosity=VISCOSITY)
cab3  = Cable(end_point1=rod3.get_endpoint_a(), end_point2=rod2.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h1, viscosity=VISCOSITY)
cab4  = Cable(end_point1=rod4.get_endpoint_a(), end_point2=rod3.get_endpoint_a(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h1, viscosity=VISCOSITY)

# Lower cables
cab5  = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod2.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h2, viscosity=VISCOSITY)
cab6  = Cable(end_point1=rod2.get_endpoint_b(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h2, viscosity=VISCOSITY)
cab7  = Cable(end_point1=rod3.get_endpoint_b(), end_point2=rod4.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h2, viscosity=VISCOSITY)
cab8  = Cable(end_point1=rod4.get_endpoint_b(), end_point2=rod1.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_h2, viscosity=VISCOSITY)


# Side cables
cab9   = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod2.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_v, viscosity=VISCOSITY)
cab10  = Cable(end_point1=rod2.get_endpoint_a(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_v, viscosity=VISCOSITY)
cab11  = Cable(end_point1=rod3.get_endpoint_a(), end_point2=rod4.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_v, viscosity=VISCOSITY)
cab12  = Cable(end_point1=rod4.get_endpoint_a(), end_point2=rod1.get_endpoint_b(), stiffness=STIFFNESS, unstretched_length=UNSTRETCHED_LENGTH_v, viscosity=VISCOSITY)


robot = TensegrityRobot()
robot.add_rods([rod1, rod2, rod3, rod4])
robot.add_cables([cab1, cab2, cab3, cab4, cab5, cab6, cab7, cab8, cab9, cab10, cab11, cab12])



rob_vis.plot_cur_state(robot)
hist_states = run_simulation(robot, time=3, dt=0.005)
rob_vis.plot_cur_state(robot)

rob_vis.plot_com_graphs(hist_states)
#rob_vis.animate_historical_states(robot=robot, states=hist_states, interval=0.01)

