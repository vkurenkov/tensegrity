import numpy         as np
import visualisation as rob_vis

from model                   import Rod, RodState, Cable, TensegrityRobot
from copy                    import deepcopy
from scipy.spatial.transform import Rotation
from math                    import cos, sin, pi, radians

DEFAULT_CONFIG = {
    "Offset": 0.3 / 8.0,
    "CableV_UnstretchedLength": 0.2,
    "CableH1_UnstretchedLength": 0.03,
    "CableH2_UnstretchedLength": 0.15,
    "Cable_Stiffness": 10,
    "Cable_Viscosity": 1,
    "Rod_Length": 0.3,
    "Rod_Mass":   1,
}

def build(n=4, config=DEFAULT_CONFIG):
    # Parse config
    length       = config["Rod_Length"]
    mass         = config["Rod_Mass"]
    offset_len   = config["Offset"]
    unstr_len_v  = config["CableV_UnstretchedLength"]
    unstr_len_h1 = config["CableH1_UnstretchedLength"]
    unstr_len_h2 = config["CableH2_UnstretchedLength"]
    stiffness    = config["Cable_Stiffness"]
    viscosity    = config["Cable_Viscosity"]

    # Degree/radian of rotation for every rod
    rot_deg = 360 / n
    rot_rad = radians(rot_deg)

    # Configure the first rod
    rot     = Rotation.from_euler("xyz", [0, -30, 0], degrees=True) * Rotation.from_euler("xyz", [0, 90, 0], degrees=True)
    offset  = (offset_len*cos(pi/n), offset_len*sin(pi/n))
    rods    = [Rod(mass=mass, inertia=np.eye(3), length=length, state=RodState(r=np.array([offset[0], offset[1], 0]), q=rot))]

    # Configure other rods with respect to the first rod
    for i in range(1, n):
        rot = Rotation.from_euler("xyz", [0, 0, rot_deg], degrees=True) * rot
        offset = (offset_len*cos(i*rot_rad+pi/n), offset_len*sin(i*rot_rad+pi/n))
        z = 0.0
        if i == 2:
            z += 0.1
        rods.append(Rod(mass=mass, inertia=np.eye(3), length=length, state=RodState(r=np.array([offset[0], offset[1], z]), q=rot)))

    # Attach the cables
    cabs = []
    for i in range(n):
        ind_to = (i + 1) % n

        # Horizontal cables (the ones that are parallel to the xy-plane)
        # Upper ones
        cabs.append(Cable(end_point1=rods[i].get_endpoint_a(), end_point2=rods[i-1].get_endpoint_a(),    stiffness=stiffness, unstretched_length=unstr_len_h1, viscosity=viscosity))
        # Lower ones
        cabs.append(Cable(end_point1=rods[i].get_endpoint_b(), end_point2=rods[ind_to].get_endpoint_b(), stiffness=stiffness, unstretched_length=unstr_len_h2, viscosity=viscosity))
        # Vertical cables (the ones that perpendicular to the xy-plane)
        cabs.append(Cable(end_point1=rods[i].get_endpoint_a(), end_point2=rods[ind_to].get_endpoint_b(), stiffness=stiffness, unstretched_length=unstr_len_v, viscosity=viscosity))

    # Build the robot
    robot = TensegrityRobot()
    robot.add_rods(rods)
    robot.add_cables(cabs)

    return robot

