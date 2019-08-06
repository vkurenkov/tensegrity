import numpy as np

class Gravity:
    def __init__(self, rod, gravity=np.array([0, 0, -9.8])):
        self._gravity = gravity
        self._rod     = rod

    def get_force(self):
        return self._rod.get_state().r, self._gravity * self._rod.get_mass()