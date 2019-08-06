import numpy         as np

#from model import Rod, RodState, Cable, Rotor



def controller(rotors):

    v = np.zeros((3, len(rotors)))
    for i in range(0, len(rotors)):

        v[:, i] = rotors[i].get_vector()

        rotors[i].set_thrust(10)
    print(v)

