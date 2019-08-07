import numpy         as np

#from model import Rod, RodState, Cable, Rotor

class QuadrotorController:
    def __init__(self, rotors):
        self._rotors = rotors
    def get_rotors(self):
        return self._rotors
    def update(self, robot):
        CoM = robot.get_center_of_mass()
        g = robot.get_mass()*9.8

        rotors = self.get_rotors()

        Av = np.zeros((3, len(rotors)))
        Ar = np.zeros((3, len(rotors)))
        for i in range(0, len(rotors)):

            v = rotors[i].get_vector()
            r = rotors[i].get_application_point_abs() - CoM
            Av[:, i] = v
            Ar[:, i] = np.cross(r, v)

        A = np.concatenate((Av, Ar))

        desired_wrench = np.array([0, 0, g, 0, 0, 0])
        u = np.linalg.pinv(A).dot(desired_wrench)

        for i in range(0, len(rotors)):
            rotors[i].set_thrust(u[i])

