
import numpy         as np
import visualisation as rob_vis

from model      import Rod, RodState, Cable, TensegrityRobot
from simulation import run_simulation


from copy import deepcopy
from scipy.spatial.transform import Rotation

np.set_printoptions(precision=5)
np.set_printoptions(suppress=True)

LENGTH = 0.2
OFFSET = LENGTH / 8.0
UNSTRETCHED_LENGTH = 0.05
STIFFNESS = 100
VISCOSITY = 0.1
MASS = 1
DELTA_L = UNSTRETCHED_LENGTH/8

rod1 = Rod(mass=MASS, inertia=np.eye(3), length=LENGTH,
           state=RodState(r=np.array([0, 0, -OFFSET]), q=Rotation.from_euler("xyz", [0, 0, 0], degrees=True)))
rod2 = Rod(mass=MASS, inertia=np.eye(3), length=LENGTH,
           state=RodState(r=np.array([0, 0, OFFSET]), q=Rotation.from_euler("xyz", [0, 0, 0], degrees=True)))

rod3 = Rod(mass=MASS, inertia=np.eye(3), length=LENGTH*3,
           state=RodState(r=np.array([-OFFSET, 0, 0]), q=Rotation.from_euler("xyz", [0, 0, 90], degrees=True)))
rod4 = Rod(mass=MASS, inertia=np.eye(3), length=LENGTH*3,
           state=RodState(r=np.array([OFFSET, 0, 0]), q=Rotation.from_euler("xyz", [0, 0, 90], degrees=True)))

rod5 = Rod(mass=MASS, inertia=np.eye(3), length=LENGTH*3,
           state=RodState(r=np.array([0, -OFFSET, 0]), q=Rotation.from_euler("xyz", [0, 90, 0], degrees=True)))
rod6 = Rod(mass=MASS, inertia=np.eye(3), length=LENGTH*3,
           state=RodState(r=np.array([0, OFFSET, 0]), q=Rotation.from_euler("xyz", [0, 90, 0], degrees=True)))

rod7 = Rod(mass=MASS, inertia=np.eye(3), length=LENGTH*3,
           state=RodState(r=np.array([0, OFFSET, 0]), q=Rotation.from_euler("xyz", [0, 90, 0], degrees=True)))

rod8 = Rod(mass=MASS, inertia=np.eye(3), length=LENGTH,
           state=RodState(r=np.array([0, OFFSET, -OFFSET]), q=Rotation.from_euler("xyz", [0, 0, 0], degrees=True)),fixed=True)
rod9 = Rod(mass=MASS, inertia=np.eye(3), length=LENGTH,
           state=RodState(r=np.array([0, OFFSET, OFFSET]), q=Rotation.from_euler("xyz", [0, 0, 0], degrees=True)),fixed=True)


cab1 = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod3.get_endpoint_a(), stiffness=STIFFNESS,
             unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab2 = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS,
             unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab3 = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod6.get_endpoint_b(), stiffness=STIFFNESS,
             unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab4 = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod5.get_endpoint_b(), stiffness=STIFFNESS,
             unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab5 = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod4.get_endpoint_a(), stiffness=STIFFNESS,
             unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab6 = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod4.get_endpoint_b(), stiffness=STIFFNESS,
             unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab7 = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod6.get_endpoint_b(), stiffness=STIFFNESS,
             unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab8 = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod5.get_endpoint_b(), stiffness=STIFFNESS,
             unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)

cab9 = Cable(end_point1=rod5.get_endpoint_b(), end_point2=rod3.get_endpoint_a(), stiffness=STIFFNESS,
             unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab10 = Cable(end_point1=rod5.get_endpoint_b(), end_point2=rod4.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab11 = Cable(end_point1=rod6.get_endpoint_b(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab12 = Cable(end_point1=rod6.get_endpoint_b(), end_point2=rod4.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab13 = Cable(end_point1=rod3.get_endpoint_a(), end_point2=rod2.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab14 = Cable(end_point1=rod3.get_endpoint_a(), end_point2=rod5.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab15 = Cable(end_point1=rod4.get_endpoint_a(), end_point2=rod2.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab16 = Cable(end_point1=rod4.get_endpoint_a(), end_point2=rod5.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab17 = Cable(end_point1=rod3.get_endpoint_b(), end_point2=rod2.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab18 = Cable(end_point1=rod3.get_endpoint_b(), end_point2=rod6.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab19 = Cable(end_point1=rod4.get_endpoint_b(), end_point2=rod6.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab20 = Cable(end_point1=rod4.get_endpoint_b(), end_point2=rod2.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab21 = Cable(end_point1=rod2.get_endpoint_a(), end_point2=rod6.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab22 = Cable(end_point1=rod2.get_endpoint_a(), end_point2=rod5.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab23 = Cable(end_point1=rod2.get_endpoint_b(), end_point2=rod6.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab24 = Cable(end_point1=rod2.get_endpoint_b(), end_point2=rod5.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)

cab25 = Cable(end_point1=rod7.get_endpoint_a(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab26 = Cable(end_point1=rod7.get_endpoint_a(), end_point2=rod4.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab27 = Cable(end_point1=rod7.get_endpoint_b(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab28 = Cable(end_point1=rod7.get_endpoint_b(), end_point2=rod4.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)

cab29 = Cable(end_point1=rod3.get_endpoint_b(), end_point2=rod8.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab30 = Cable(end_point1=rod3.get_endpoint_b(), end_point2=rod9.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab31 = Cable(end_point1=rod4.get_endpoint_b(), end_point2=rod8.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab32 = Cable(end_point1=rod4.get_endpoint_b(), end_point2=rod9.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)

cab33 = Cable(end_point1=rod7.get_endpoint_a(), end_point2=rod9.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab34 = Cable(end_point1=rod7.get_endpoint_a(), end_point2=rod9.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab35 = Cable(end_point1=rod7.get_endpoint_b(), end_point2=rod8.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab36 = Cable(end_point1=rod7.get_endpoint_b(), end_point2=rod8.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)

cab37 = Cable(end_point1=rod6.get_endpoint_a(), end_point2=rod9.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab38 = Cable(end_point1=rod6.get_endpoint_a(), end_point2=rod9.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab39 = Cable(end_point1=rod6.get_endpoint_b(), end_point2=rod8.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)
cab40 = Cable(end_point1=rod6.get_endpoint_b(), end_point2=rod8.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY)


robot = TensegrityRobot()
robot.add_rods([rod1, rod2, rod3, rod4, rod5, rod6, rod7, rod8, rod9])
robot.add_cables([ cab1, cab3, cab4, cab5, cab7, cab8, cab9, cab10])
robot.add_cables([cab13, cab14, cab15, cab16,   cab21, cab22, cab23, cab24])
robot.add_cables([cab25, cab26, cab27, cab28, cab29, cab30, cab31, cab32, cab33, cab34, cab35, cab36])
robot.add_cables([cab37, cab38, cab39, cab40])
rob_vis.plot_cur_state(robot)

hist_states = run_simulation(robot, time=10, dt=0.01)
rob_vis.animate_historical_states(robot=robot, states=hist_states, interval=0.01)

'''
pos0 = robot.get_rods()[4].get_endpoint_a().get_position()
K = []
for i in robot.get_cables():
    print("1", i.get_unstretched_length())
    len = i.get_unstretched_length()
    i.set_unstretched_length(len /2)
    hist_states = run_simulation( robot, time=2, dt=0.01 )
    rob_vis.animate_historical_states( robot=robot, states=hist_states, interval=0.01 )
    print("2", i.get_unstretched_length() )
    print(robot.get_rods()[4].get_endpoint_a().get_position() - pos0)
    i.set_unstretched_length(len)
    print( "3",i.get_unstretched_length() )
K = np.array(K).transpose() / DELTA_L
print(K)
rob_vis.plot_cur_state(robot)

dl = 0.01
K = 
[[-0.  -0.1  0.1  0.   0.1 -0.1  0.1 -0.1 -0.  -0.   0.   0.  -0.  -0.
  -0.  -0.  -0.   0.   0.  -0.  -0.   0.  -0.   0.   0.   0.1 -0.  -0.
  -0.   0.  -0.   0.  -0.   0.  -0.   0. ]
 [-0.1 -0.2 -0.3 -0.2 -0.3 -0.4 -0.2 -0.2 -0.4 -0.4 -0.5 -0.4 -0.6 -0.5
  -0.6 -0.6 -0.7 -0.7 -0.7 -0.8 -0.8 -0.8 -0.8 -0.8 -0.8 -0.8 -0.8 -0.9
  -0.9 -1.  -1.  -1.1 -1.1 -1.2 -1.2 -1.3]
 [ 0.1  0.1  0.   0.1  0.1  0.1  0.  -0.   0.1  0.1  0.1  0.3  0.1  0.3
   0.2  0.1  0.1  0.2  0.2  0.3  0.1  0.1  0.2  0.2  0.2  0.2  0.2  0.2
   0.2  0.2  0.2  0.2  0.2  0.2  0.2  0.2]]
dl = 0.001
K = 
[[ -0.   -0.1   0.1   0.    0.1  -0.1   0.1  -0.1  -0.   -0.    0.    0.
   -0.   -0.   -0.    0.   -0.    0.    0.   -0.   -0.    0.   -0.    0.
    0.    0.1  -0.   -0.   -0.    0.   -0.    0.   -0.    0.   -0.    0. ]
 [ -1.1  -1.6  -2.   -2.2  -2.6  -3.1  -3.2  -3.5  -4.   -4.3  -4.7  -4.9
   -5.3  -5.5  -5.9  -6.2  -6.6  -6.9  -7.2  -7.5  -7.8  -8.1  -8.3  -8.6
   -8.9  -9.1  -9.4  -9.7 -10.  -10.3 -10.6 -10.8 -11.1 -11.4 -11.7 -11.9]
 [  0.5   0.7   0.7   0.8   0.9   0.9   0.9   0.9   1.1   1.1   1.2   1.4
    1.3   1.5   1.4   1.4   1.5   1.6   1.5   1.7   1.6   1.6   1.7   1.8
    1.8   1.8   1.9   1.9   1.9   2.    2.    2.    2.    2.1   2.1   2.1]]
dl = 0.0001
    K = 
[[  -0.    -0.1    0.1    0.     0.1   -0.1    0.1   -0.1    0.     0.
     0.     0.     0.    -0.     0.     0.     0.     0.1    0.    -0.
    -0.     0.    -0.     0.     0.     0.1    0.    -0.     0.     0.
     0.     0.     0.     0.     0.     0. ]
 [ -10.9  -15.2  -18.8  -22.2  -25.8  -29.4  -32.7  -36.1  -39.7  -43.
   -46.4  -49.6  -52.9  -56.1  -59.3  -62.5  -65.6  -68.7  -71.8  -74.8
   -77.7  -80.7  -83.6  -86.5  -89.3  -92.   -94.8  -97.6 -100.4 -103.1
  -105.8 -108.4 -111.  -113.7 -116.2 -118.7]
 [   5.3    6.6    7.3    7.9    8.5    9.1    9.6   10.1   10.7   11.3
    11.8   12.4   12.8   13.4   13.8   14.2   14.6   15.2   15.5   16.
    16.3   16.6   17.1   17.5   17.8   18.1   18.5   18.8   19.1   19.5
    19.8   20.    20.3   20.6   20.9   21.1]]
'''