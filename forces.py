import numpy as np

from copy import copy

class Gravity:
    def __init__(self, rod, gravity=np.array([0, 0, -9.8])):
        self._gravity = gravity
        self._rod     = rod

    def get_force(self):
        return self._rod.get_state().r, self._gravity * self._rod.get_mass()

class Rotor:
    def __init__(self, EndPoint):
        self._EndPoint = EndPoint
        self._thrust = 0
    def set_thrust(self, thrust):
        self._thrust = thrust
    def get_thrust(self):
        return copy(self._thrust)
    def get_vector(self):
        #return np.array([[1], [0], [0]])
        return np.array([0, 0, 1])
    def get_force(self):
        F = self.get_vector() * self.get_thrust()
        r = self._EndPoint.get_position()
        return r, F