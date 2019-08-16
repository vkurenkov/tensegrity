import numpy         as np
from math import cos, sin, pi
import visualisation as rob_vis

from model                   import Rod, RodState, Cable, TensegrityRobot
from simulation              import run_simulation

from copy                    import deepcopy
from scipy.spatial.transform import Rotation

LENGTH = 0.3
OFFSET = LENGTH/4.0
UNSTRETCHED_LENGTH_v = 0.2
UNSTRETCHED_LENGTH_h1 = 0.1
UNSTRETCHED_LENGTH_h2 = 0.1
STIFFNESS1 = 20
STIFFNESS2 = 20
VISCOSITY = 0.1
VISCOSITY_W = 0.1
MASS = 1
INERTIA = np.diag([(1/12)*MASS*LENGTH**2, (1/12)*MASS*LENGTH**2, (1/12)*MASS*LENGTH**2])

q0=Rotation.from_euler("xyz", [0, 90, 0], degrees=True)

q1= Rotation.from_euler("xyz", [0, 0, 0], degrees=True) * q0
q2= Rotation.from_euler("xyz", [0, 0, 90], degrees=True) * q1
q3= Rotation.from_euler("xyz", [0, 0, 90], degrees=True) * q2
q4= Rotation.from_euler("xyz", [0, 0, 90], degrees=True) * q3

rod1  = Rod(mass=MASS, inertia=INERTIA, length=LENGTH, state=RodState(r=np.array([ OFFSET*cos(0*pi/2+pi/4),  OFFSET*sin(0*pi/2+pi/4), 0]), q=q1), viscosity_w=VISCOSITY_W)
rod2  = Rod(mass=1, inertia=INERTIA, length=LENGTH, state=RodState(r=np.array([ OFFSET*cos(1*pi/2+pi/4),  OFFSET*sin(1*pi/2+pi/4), 0]), q=q2),    viscosity_w=VISCOSITY_W)
rod3  = Rod(mass=1, inertia=INERTIA, length=LENGTH, state=RodState(r=np.array([ OFFSET*cos(2*pi/2+pi/4),  OFFSET*sin(2*pi/2+pi/4), 0]), q=q3),    viscosity_w=VISCOSITY_W)
rod4  = Rod(mass=1, inertia=INERTIA, length=LENGTH, state=RodState(r=np.array([ OFFSET*cos(3*pi/2+pi/4),  OFFSET*sin(3*pi/2+pi/4), 0]), q=q4),    viscosity_w=VISCOSITY_W)

rod1.set_name("Rod1")

# Upper cables
cab1  = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod4.get_endpoint_a(), stiffness=STIFFNESS2, unstretched_length=UNSTRETCHED_LENGTH_h1, viscosity=VISCOSITY)
cab2  = Cable(end_point1=rod2.get_endpoint_a(), end_point2=rod1.get_endpoint_a(), stiffness=STIFFNESS2, unstretched_length=UNSTRETCHED_LENGTH_h1, viscosity=VISCOSITY)
cab3  = Cable(end_point1=rod3.get_endpoint_a(), end_point2=rod2.get_endpoint_a(), stiffness=STIFFNESS2, unstretched_length=UNSTRETCHED_LENGTH_h1, viscosity=VISCOSITY)
cab4  = Cable(end_point1=rod4.get_endpoint_a(), end_point2=rod3.get_endpoint_a(), stiffness=STIFFNESS2, unstretched_length=UNSTRETCHED_LENGTH_h1, viscosity=VISCOSITY)

# Lower cables
cab5  = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod2.get_endpoint_b(), stiffness=STIFFNESS2, unstretched_length=UNSTRETCHED_LENGTH_h2, viscosity=VISCOSITY)
cab6  = Cable(end_point1=rod2.get_endpoint_b(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS2, unstretched_length=UNSTRETCHED_LENGTH_h2, viscosity=VISCOSITY)
cab7  = Cable(end_point1=rod3.get_endpoint_b(), end_point2=rod4.get_endpoint_b(), stiffness=STIFFNESS2, unstretched_length=UNSTRETCHED_LENGTH_h2, viscosity=VISCOSITY)
cab8  = Cable(end_point1=rod4.get_endpoint_b(), end_point2=rod1.get_endpoint_b(), stiffness=STIFFNESS2, unstretched_length=UNSTRETCHED_LENGTH_h2, viscosity=VISCOSITY)


# Side cables
cab9   = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod2.get_endpoint_b(), stiffness=STIFFNESS1, unstretched_length=UNSTRETCHED_LENGTH_v, viscosity=VISCOSITY)
cab10  = Cable(end_point1=rod2.get_endpoint_a(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS1, unstretched_length=UNSTRETCHED_LENGTH_v, viscosity=VISCOSITY)
cab11  = Cable(end_point1=rod3.get_endpoint_a(), end_point2=rod4.get_endpoint_b(), stiffness=STIFFNESS1, unstretched_length=UNSTRETCHED_LENGTH_v, viscosity=VISCOSITY)
cab12  = Cable(end_point1=rod4.get_endpoint_a(), end_point2=rod1.get_endpoint_b(), stiffness=STIFFNESS1, unstretched_length=UNSTRETCHED_LENGTH_v, viscosity=VISCOSITY)


robot = TensegrityRobot()
robot.add_rods([rod1, rod2, rod3, rod4])
robot.add_cables([cab1, cab2, cab3, cab4, cab5, cab6, cab7, cab8, cab9, cab10, cab11, cab12])



rob_vis.plot_cur_state(robot)
hist_states = run_simulation(robot, time=1, dt=0.002)
# hist_states = run_simulation(robot, time=0.3, dt=0.0005)
rob_vis.plot_cur_state(robot)

# rob_vis.plot_com_graphs(hist_states)
#rob_vis.animate_historical_states(robot=robot, states=hist_states, interval=0.01)

