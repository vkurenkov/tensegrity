import numpy         as np
import visualisation as rob_vis

from model import Rod, RodState, Cable, TensegrityRobot
from simulation import run_simulation

from copy import deepcopy
from scipy.spatial.transform import Rotation

np.set_printoptions( precision=5 )
np.set_printoptions( suppress=True )

LENGTH = 0.1
OFFSET = LENGTH / 8.0
UNSTRETCHED_LENGTH = 0.05
STIFFNESS = 1
VISCOSITY_DR = 1
VISCOSITY_W = 1
VISCOSITY = 1
MASS = 0.8
INERTIA = np.diag( [ (1 / 12) * MASS * LENGTH ** 2, (1 / 12) * MASS * LENGTH ** 2, (1 / 12) * MASS * LENGTH ** 2 ] )
DELTA_L = 0.001

rod1 = Rod( mass=MASS, inertia=INERTIA, length=LENGTH,
            state=RodState( r=np.array( [ 0, 0, -OFFSET ] ),
                            q=Rotation.from_euler( "xyz", [ 0, 0, 0 ],
                                                   degrees=True ) ),
            viscosity_dr=VISCOSITY_DR,
            viscosity_w=VISCOSITY_W )
rod2 = Rod( mass=MASS, inertia=INERTIA, length=LENGTH,
            state=RodState( r=np.array( [ 0, 0, OFFSET ] ), q=Rotation.from_euler( "xyz", [ 0, 0, 0 ], degrees=True ) ),
            viscosity_dr=VISCOSITY_DR, viscosity_w=VISCOSITY_W )

rod3 = Rod( mass=MASS, inertia=INERTIA, length=LENGTH * 3, state=RodState( r=np.array( [ -OFFSET, 0, 0 ] ),
                                                                           q=Rotation.from_euler( "xyz", [ 0, 0, 90 ],
                                                                                                  degrees=True ) ),
            viscosity_dr=VISCOSITY_DR,
            viscosity_w=VISCOSITY_W )
rod4 = Rod( mass=MASS, inertia=INERTIA, length=LENGTH * 3, state=RodState( r=np.array( [ OFFSET, 0, 0 ] ),
                                                                           q=Rotation.from_euler( "xyz", [ 0, 0, 90 ],
                                                                                                  degrees=True ) ),
            viscosity_dr=VISCOSITY_DR,
            viscosity_w=VISCOSITY_W )

rod5 = Rod( mass=MASS, inertia=INERTIA, length=LENGTH * 3, state=RodState( r=np.array( [ 0, -OFFSET, 0 ] ),
                                                                           q=Rotation.from_euler( "xyz", [ 0, 90, 0 ],
                                                                                                  degrees=True ) ),
            viscosity_dr=VISCOSITY_DR,
            viscosity_w=VISCOSITY_W )
rod6 = Rod( mass=MASS, inertia=INERTIA, length=LENGTH, state=RodState( r=np.array( [ 0, OFFSET, 0 ] ),
                                                                       q=Rotation.from_euler( "xyz", [ 0, 90, 0 ],
                                                                                              degrees=True ) ),
            viscosity_dr=VISCOSITY_DR,
            viscosity_w=VISCOSITY_W )

rod7 = Rod( mass=MASS, inertia=INERTIA, length=LENGTH,
            state=RodState( r=np.array( [ 0, OFFSET * 10, 0 ] ),
                            q=Rotation.from_euler( "xyz", [ 0, 90, 0 ], degrees=True ) ), viscosity_dr=VISCOSITY_DR,
            viscosity_w=VISCOSITY_W )

rod8 = Rod( mass=MASS, inertia=INERTIA, length=LENGTH,
            state=RodState( r=np.array( [ 0, OFFSET * 8, -OFFSET ] ),
                            q=Rotation.from_euler( "xyz", [ 0, 0, 0 ], degrees=True ) ), viscosity_dr=VISCOSITY_DR,
            viscosity_w=VISCOSITY_W ,fixed=True)
rod9 = Rod( mass=MASS, inertia=INERTIA, length=LENGTH,
            state=RodState( r=np.array( [ 0, OFFSET * 8, OFFSET ] ),
                            q=Rotation.from_euler( "xyz", [ 0, 0, 0 ], degrees=True ) ), viscosity_dr=VISCOSITY_DR,
            viscosity_w=VISCOSITY_W ,fixed=True)

cab1 = Cable( end_point1=rod1.get_endpoint_a(), end_point2=rod3.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab2 = Cable( end_point1=rod1.get_endpoint_a(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH * 4, viscosity=VISCOSITY )
cab3 = Cable( end_point1=rod1.get_endpoint_a(), end_point2=rod6.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab4 = Cable( end_point1=rod1.get_endpoint_a(), end_point2=rod5.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab5 = Cable( end_point1=rod1.get_endpoint_b(), end_point2=rod4.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab6 = Cable( end_point1=rod1.get_endpoint_b(), end_point2=rod4.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH * 4, viscosity=VISCOSITY )
cab7 = Cable( end_point1=rod1.get_endpoint_b(), end_point2=rod6.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab8 = Cable( end_point1=rod1.get_endpoint_b(), end_point2=rod5.get_endpoint_b(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )

cab9 = Cable( end_point1=rod5.get_endpoint_b(), end_point2=rod3.get_endpoint_a(), stiffness=STIFFNESS,
              unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab10 = Cable( end_point1=rod5.get_endpoint_b(), end_point2=rod4.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab11 = Cable( end_point1=rod6.get_endpoint_b(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH * 4, viscosity=VISCOSITY )
cab12 = Cable( end_point1=rod6.get_endpoint_b(), end_point2=rod4.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH * 4, viscosity=VISCOSITY )
cab13 = Cable( end_point1=rod3.get_endpoint_a(), end_point2=rod2.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab14 = Cable( end_point1=rod3.get_endpoint_a(), end_point2=rod5.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH * 4, viscosity=VISCOSITY )
cab15 = Cable( end_point1=rod4.get_endpoint_a(), end_point2=rod2.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab16 = Cable( end_point1=rod4.get_endpoint_a(), end_point2=rod5.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH * 4, viscosity=VISCOSITY )
cab17 = Cable( end_point1=rod3.get_endpoint_b(), end_point2=rod2.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH * 4, viscosity=VISCOSITY )
cab18 = Cable( end_point1=rod3.get_endpoint_b(), end_point2=rod6.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH * 4, viscosity=VISCOSITY )
cab19 = Cable( end_point1=rod4.get_endpoint_b(), end_point2=rod6.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH * 4, viscosity=VISCOSITY )
cab20 = Cable( end_point1=rod4.get_endpoint_b(), end_point2=rod2.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH * 4, viscosity=VISCOSITY )
cab21 = Cable( end_point1=rod2.get_endpoint_a(), end_point2=rod6.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab22 = Cable( end_point1=rod2.get_endpoint_a(), end_point2=rod5.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH * 4, viscosity=VISCOSITY )
cab23 = Cable( end_point1=rod2.get_endpoint_b(), end_point2=rod6.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab24 = Cable( end_point1=rod2.get_endpoint_b(), end_point2=rod5.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH * 4, viscosity=VISCOSITY )

cab25 = Cable( end_point1=rod7.get_endpoint_a(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab26 = Cable( end_point1=rod7.get_endpoint_a(), end_point2=rod4.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab27 = Cable( end_point1=rod7.get_endpoint_b(), end_point2=rod3.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab28 = Cable( end_point1=rod7.get_endpoint_b(), end_point2=rod4.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )

cab29 = Cable( end_point1=rod3.get_endpoint_b(), end_point2=rod8.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab30 = Cable( end_point1=rod3.get_endpoint_b(), end_point2=rod9.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab31 = Cable( end_point1=rod4.get_endpoint_b(), end_point2=rod8.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab32 = Cable( end_point1=rod4.get_endpoint_b(), end_point2=rod9.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )

cab33 = Cable( end_point1=rod7.get_endpoint_a(), end_point2=rod9.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab34 = Cable( end_point1=rod7.get_endpoint_a(), end_point2=rod9.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab35 = Cable( end_point1=rod7.get_endpoint_b(), end_point2=rod8.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )
cab36 = Cable( end_point1=rod7.get_endpoint_b(), end_point2=rod8.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH / 4, viscosity=VISCOSITY )

cab37 = Cable( end_point1=rod6.get_endpoint_a(), end_point2=rod9.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY )
cab38 = Cable( end_point1=rod6.get_endpoint_a(), end_point2=rod9.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY )
cab39 = Cable( end_point1=rod6.get_endpoint_b(), end_point2=rod8.get_endpoint_a(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY )
cab40 = Cable( end_point1=rod6.get_endpoint_b(), end_point2=rod8.get_endpoint_b(), stiffness=STIFFNESS,
               unstretched_length=UNSTRETCHED_LENGTH, viscosity=VISCOSITY )

robot = TensegrityRobot()
robot.add_rods( [ rod1, rod2, rod3, rod4, rod5, rod6, rod7, rod8, rod9 ] )
robot.add_cables( [ cab1, cab3, cab4, cab5, cab7, cab8, cab9, cab10, cab11, cab12 ] )
robot.add_cables( [ cab13, cab14, cab15, cab16, cab18, cab19, cab21, cab22, cab23, cab24 ] )
robot.add_cables( [ cab25, cab26, cab27, cab28, cab29, cab30, cab31, cab32, cab33, cab34, cab35, cab36 ] )
robot.add_cables( [ cab37, cab38, cab39, cab40 ] )
# rob_vis.plot_cur_state(robot)

hist_states = run_simulation( robot, time=7, dt=0.001 )
print(robot.get_kinetic_energy())
#rob_vis.animate_historical_states( robot=robot, states=hist_states, interval=0.1 )
pos0 = robot.get_rods()[4].get_endpoint_a().get_position()
K = []
for i in robot.get_cables():
    print("1", i.get_unstretched_length())
    len = i.get_unstretched_length()
    #i.set_unstretched_length(len + DELTA_L)
    hist_states = run_simulation( robot, time=1, dt=0.001 )
    #rob_vis.animate_historical_states( robot=robot, states=hist_states, interval=0.1 )
    print("2", i.get_unstretched_length() )
    K.append(robot.get_rods()[4].get_endpoint_a().get_position() - pos0)
    #i.set_unstretched_length(len)
    print( "3",i.get_unstretched_length() )
K = np.array(K).transpose() / DELTA_L
print(K)
rob_vis.plot_cur_state(robot)
