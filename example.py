import numpy         as np
import visualisation as rob_vis

from model                   import Rod, RodState, Cable, TensegrityRobot
from copy                    import deepcopy
from scipy.spatial.transform import Rotation

LENGTH = 5.0
OFFSET = LENGTH/4.0

rod1  = Rod(mass=1, inertia=np.eye(1), length=LENGTH, state=RodState(r=np.array([0, 0, -OFFSET]), q=Rotation.from_euler("xyz", [0, 0, 0], degrees=True)))
rod2  = Rod(mass=1, inertia=np.eye(1), length=LENGTH, state=RodState(r=np.array([0, 0, OFFSET]),  q=Rotation.from_euler("xyz", [0, 0, 0], degrees=True)))
rod3  = Rod(mass=1, inertia=np.eye(1), length=LENGTH, state=RodState(r=np.array([-OFFSET, 0, 0]), q=Rotation.from_euler("xyz", [0, 0, 90], degrees=True)))
rod4  = Rod(mass=1, inertia=np.eye(1), length=LENGTH, state=RodState(r=np.array([OFFSET, 0, 0]),  q=Rotation.from_euler("xyz", [0, 0, 90], degrees=True)))
rod5  = Rod(mass=1, inertia=np.eye(1), length=LENGTH, state=RodState(r=np.array([0, -OFFSET, 0]), q=Rotation.from_euler("xyz", [0, 90, 0], degrees=True)))
rod6  = Rod(mass=1, inertia=np.eye(1), length=LENGTH, state=RodState(r=np.array([0, OFFSET, 0]),  q=Rotation.from_euler("xyz", [0, 90, 0], degrees=True)))

cab1  = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod3.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab2  = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod3.get_endpoint_b(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab3  = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod6.get_endpoint_b(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab4  = Cable(end_point1=rod1.get_endpoint_a(), end_point2=rod5.get_endpoint_b(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab5  = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod4.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab6  = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod4.get_endpoint_b(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab7  = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod6.get_endpoint_b(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab8  = Cable(end_point1=rod1.get_endpoint_b(), end_point2=rod5.get_endpoint_b(), stiffness=10, unstretched_length=0.1, viscosity=10)

cab9  = Cable(end_point1=rod5.get_endpoint_b(), end_point2=rod3.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab10 = Cable(end_point1=rod5.get_endpoint_b(), end_point2=rod4.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab11  = Cable(end_point1=rod6.get_endpoint_b(), end_point2=rod3.get_endpoint_b(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab12 = Cable(end_point1=rod6.get_endpoint_b(), end_point2=rod4.get_endpoint_b(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab13  = Cable(end_point1=rod3.get_endpoint_a(), end_point2=rod2.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab14 = Cable(end_point1=rod3.get_endpoint_a(), end_point2=rod5.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab15  = Cable(end_point1=rod4.get_endpoint_a(), end_point2=rod2.get_endpoint_b(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab16 = Cable(end_point1=rod4.get_endpoint_a(), end_point2=rod5.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab17  = Cable(end_point1=rod3.get_endpoint_b(), end_point2=rod2.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab18 = Cable(end_point1=rod3.get_endpoint_b(), end_point2=rod6.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab19  = Cable(end_point1=rod4.get_endpoint_b(), end_point2=rod6.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab20 = Cable(end_point1=rod4.get_endpoint_b(), end_point2=rod2.get_endpoint_b(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab21  = Cable(end_point1=rod2.get_endpoint_a(), end_point2=rod6.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab22 = Cable(end_point1=rod2.get_endpoint_a(), end_point2=rod5.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab23  = Cable(end_point1=rod2.get_endpoint_b(), end_point2=rod6.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)
cab24 = Cable(end_point1=rod2.get_endpoint_b(), end_point2=rod5.get_endpoint_a(), stiffness=10, unstretched_length=0.1, viscosity=10)


robot = TensegrityRobot()
robot.add_rods([rod1, rod2, rod3, rod4, rod5, rod6])
robot.add_cables([cab1, cab2, cab3, cab4, cab5, cab6, cab7, cab8, cab9, cab10, cab11, cab12])
robot.add_cables([cab13, cab14, cab15, cab16, cab17, cab18, cab19, cab20, cab21, cab22, cab23, cab24])
# print("First rod cables vectors")
# print(rod1.get_cables_states())
# print("Second rod cables vectors")
# print(rod2.get_cables_states())

# Visualise the initial state of the robot
rob_vis.plot(robot)

