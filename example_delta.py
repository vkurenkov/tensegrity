import numpy         as np
from math import cos, sin, pi
import visualisation as rob_vis

from model                   import Rod, RodState, Cable, TensegrityRobot
from simulation              import run_simulation

from copy                    import deepcopy
from scipy.spatial.transform import Rotation

np.set_printoptions(precision=5)
np.set_printoptions(suppress=True)

MASS = 0.5
LENGTH = 0.5
OFFSET = LENGTH/8.0
UNSTRETCHED_LENGTH_v = 0.3
UNSTRETCHED_LENGTH_h1 = 0.3
UNSTRETCHED_LENGTH_h2 = 0.3
STIFFNESS = 100
VISCOSITY = 5
DELTA_L = 0.0001
q0=Rotation.from_euler("xyz", [0, 90, 0], degrees=True)

q1= Rotation.from_euler("xyz", [0, -60, 0], degrees=True) * q0
q2= Rotation.from_euler("xyz", [0, 0, 90], degrees=True) * q1
q3= Rotation.from_euler("xyz", [0, 0, 90], degrees=True) * q2
q4= Rotation.from_euler("xyz", [0, 0, 90], degrees=True) * q3

I = np.array([[MASS*(3*(LENGTH/50)**2 + LENGTH**2)/12,0,0],
              [0,MASS*(3*(LENGTH/50)**2 + LENGTH**2)/12,0],
              [0,0,MASS*(LENGTH/50)**2/2]]) * 1000
I = np.eye(3)

rod1  = Rod(mass=MASS, inertia=I, length=LENGTH, state=RodState(r=np.array([ 4.31674496e-06,  8.18455267e-02, -8.42711692e-07]), q=q1))
rod2  = Rod(mass=MASS, inertia=I, length=LENGTH, state=RodState(r=np.array([-8.76066069e-02,  1.32198670e-05,  1.15859006e-02]), q=q2))
rod3  = Rod(mass=MASS, inertia=I, length=LENGTH, state=RodState(r=np.array([-4.35908120e-06, -8.18586523e-02,  6.47316936e-07]), q=q3))
rod4  = Rod(mass=MASS, inertia=I, length=LENGTH, state=RodState(r=np.array([ 8.76066492e-02, -9.42450237e-08, -1.15856952e-02]), q=q4))

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

hist_states = run_simulation(robot, time=20, dt=0.01)

x0 = np.array([ i.get_length() for i in robot.get_cables()])
K = []
for i in robot.get_cables():
    i.set_unstretched_length(i.get_unstretched_length() + DELTA_L)
    hist_states = run_simulation( robot, time=5, dt=0.01 )
    K.append(np.array([ i.get_length() for i in robot.get_cables()]) - x0)
    i.set_unstretched_length( i.get_unstretched_length() - DELTA_L)
K = np.array(K).transpose() / DELTA_L
print(K)
#rob_vis.animate_historical_states(robot=robot, states=hist_states, interval=0.005)
"""
 dl = 0.00001
 K = [[ -25.77818  -43.49958  -56.17923  -66.70387  -75.86369  -83.99805   -91.38416  -98.21841 -104.14525 -109.74069 -114.78067 -119.39992]
     [ -10.27299  -12.99077  -14.69404  -14.5472   -14.16466  -13.37786   -12.6288   -11.89215  -11.29229  -10.47179   -9.79267   -9.14192]
     [  12.40225   17.98624   21.62725   22.54353   23.69624   24.50958    25.52812   26.21032   26.8346    27.43247   28.11066   28.62299]
     [   4.12376    8.70432   13.30895   18.53393   21.9786    25.97405    29.48972   33.15169   36.28072   39.21466   41.88162   44.5015 ]
     [ -26.60826  -43.62849  -56.1498   -66.71253  -75.04955  -83.94839   -91.34225  -98.11275 -104.26919 -109.75983 -114.78221 -119.3172 ]
     [   4.03894    8.71875   13.246     17.85621   21.99388   26.71737    29.70569   33.07589   36.21643   39.12388   41.94502   44.4801 ]
     [  12.4336    17.91919   20.79542   22.39311   23.6994    24.63826    26.30046   26.30513   26.87206   27.52959   28.02036   28.61677]
     [ -10.3352   -13.75495  -14.77421  -14.65484  -14.05179  -13.35509   -12.71361  -11.07129  -11.1413   -10.4588    -9.76429   -9.19712]
     [   7.60554   10.37332   11.48685   11.88686   11.98147   12.1485    12.18796   12.26571   13.17645   12.416     12.51704   12.60557]
     [   3.18543    3.39316    2.4433     1.35714    0.21184   -0.88536    -1.74367   -2.58457   -3.32076   -3.08232   -4.48969   -4.95714]
     [   3.17767    3.37066    2.52657    1.30402    0.21893   -0.82729    -1.81843   -2.56405   -3.31862   -3.94619   -3.62904   -4.96524]
     [   7.5184    10.43931   11.49215   11.91556   12.06389   12.12152    12.18995   12.19678   12.31231   12.42117   12.51117   13.46544]]
 dl = 0.0001
 K = 
 dl = 0.0005
 K = [[-0.26748 -0.0152  -0.08501  0.05047  0.25348 -0.06833  0.07371  0.01062 -0.07006  0.03168 -0.02092  0.07828]
     [ 0.21081  0.5425   0.25751  0.16704 -0.11515 -0.09756 -0.22977 -0.14566 -0.54121 -0.58901 -0.6676  -0.75785]
     [-0.20321  0.14066  0.0843  -0.00842  0.17778  0.04295 -0.01213 -0.16882 0.29518  0.10445 -0.12149 -0.33662]]
 dl = 0.001
 K = [[-0.26775 -0.01501 -0.08514  0.05062  0.25373 -0.06851  0.07374  0.01041 -0.07017  0.03158 -0.02094  0.07828]
     [ 0.05222  0.37033  0.12541  0.09368 -0.12491 -0.04364 -0.11486  0.0272 -0.31409 -0.31098 -0.34207 -0.38814]
     [-0.20319  0.1408   0.08435 -0.00814  0.17781  0.04284 -0.01207 -0.1689 0.29527  0.1045  -0.12156 -0.33673]]
 dl = 0.01
 K = [[-0.27269 -0.01156 -0.08745  0.05349  0.25841 -0.07159  0.07434  0.00679 -0.07215  0.0301  -0.02126  0.07844]
     [-0.09153  0.21677  0.00708  0.02864 -0.13454  0.00844 -0.00922  0.18569 -0.11026 -0.06177 -0.04927 -0.05346]
     [-0.2029   0.14327  0.08512 -0.0031   0.17829  0.04099 -0.01079 -0.17012 0.2966   0.10526 -0.12279 -0.33841]]

"""