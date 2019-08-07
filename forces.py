import numpy as np
from scipy.spatial.transform import Rotation

from copy import copy

class Gravity:
    def __init__(self, rod, gravity=np.array([0, 0, -9.8])):
        self._gravity = gravity
        self._rod     = rod

    def get_force(self):
        return np.array([0.0, 0.0, 0.0]), self._gravity * self._rod.get_mass()

class Rotor:
    def __init__(self, EndPoint, orientation=Rotation.from_euler('xyz', [0, 0, 0])):
        self._EndPoint = EndPoint
        self._thrust = 0
        self._orientation = orientation
    def set_orientation(self, orientation):
        self._orientation = orientation
    def get_orientation(self):
        return copy(self._orientation)
    def set_thrust(self, thrust):
        self._thrust = thrust
    def get_thrust(self):
        return copy(self._thrust)
    def get_vector(self):
        r = np.array([-1.0, 0.0, 0.0])
        r = self._orientation.apply(r)
        r = self._EndPoint.get_rod().get_state().q.apply(r)
        return r
    def get_application_point(self):
        return self._EndPoint.get_position_relative_to_CoM()
    def get_application_point_abs(self):
        return self._EndPoint.get_position()
    def get_force(self):
        return self.get_application_point(), self.get_vector() * self.get_thrust()