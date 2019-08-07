import numpy as np

from forces                  import Gravity

from typing                  import List, Tuple, Iterable
from scipy.spatial.transform import Rotation
from dataclasses             import dataclass
from copy                    import copy, deepcopy


class TensegrityRobot:
    def __init__(self):
        self._rods   = []
        self._cables = []

    def update(self, dt=0.001):
        """
        Updates all rods.
        """
        states = []
        for rod in self.get_rods():
            states.append(rod.update_taylor(dt=dt))

        for state, rod in zip(states, self.get_rods()):
            rod.set_state(state)
    def enable_gravity(self, gravity=np.array([0, 0, -9.8])):
        for rod in self._rods:
            rod.add_force_source(Gravity(rod, gravity=gravity))

    def add_rods(self, rods):
        self._rods.extend(rods)
    def add_cables(self, cables):
        self._cables.extend(cables)
    def get_rods(self):
        return self._rods
    def get_cables(self):
        return self._cables
    def get_total_energy(self):
        K = 0
        for rod in self.get_rods():
            K = K + rod.get_kinetic_energy()

        P = 0
        for cable in self.get_cables():
            P = P + cable.get_potential_energy()

        return K + P

    def get_mass(self):
        mass = 0
        for rod in self.get_rods():
            mass = mass + rod.get_mass()
        return mass

    def get_center_of_mass(self):
        CoM = 0
        mass = 0
        for rod in self.get_rods():
            CoM = CoM + rod.get_state().r * rod.get_mass()
            mass = mass + rod.get_mass()
        return CoM / mass

    def set_state(self, state):
        for ind, rod in enumerate(self.get_rods()):
            rod.get_state().r  = state["Rod{}_CoM".format(ind)]
            rod.get_state().q  = Rotation.from_quat(state["Rod{}_Rot".format(ind)])
            rod.get_state().dr = state["Rod{}_dr".format(ind)]
            rod.get_state().w  = state["Rod{}_w".format(ind)]
    def get_state(self):
        state = {}
        for ind, rod in enumerate(self.get_rods()):
            state["Rod{}_CoM".format(ind)] = rod.get_state().r
            state["Rod{}_Rot".format(ind)] = rod.get_state().q.as_quat()
            state["Rod{}_dr".format(ind)]  = rod.get_state().dr
            state["Rod{}_w".format(ind)]   = rod.get_state().w
        return state

class RodState:
    def __init__(self, q=Rotation.from_rotvec(np.zeros(3,)), r=np.zeros(3,), w=np.zeros(3,), dr=np.zeros(3,)):
        # Orientation
        self.q  = q
        # Position
        self.r  = r
        # Angle velocity
        self.w  = w
        # Velocity
        self.dr = dr

class Rod:
    def __init__(self, mass, length, inertia, state, viscosity_dr=20, viscosity_w=20):
        self._mass       = mass
        self._length     = length
        self._inertia    = inertia
        self._state      = state
        self._viscosity_dr = viscosity_dr
        self._viscosity_w  = viscosity_w
        self._endpoint_a = EndPoint(holder=self)
        self._endpoint_b = EndPoint(holder=self)
        self._force_sources = [self._endpoint_a, self._endpoint_b]

    # Topological properties
    def get_endpoint_a(self):
        return self._endpoint_a
    def get_endpoint_b(self):
        return self._endpoint_b
    def get_cables(self):
        cables = self.get_endpoint_a().get_cables()
        cables.extend(self.get_endpoint_b().get_cables())
        return cables
    
    # Geometrical properties
    def get_cables_states(self, state=None):
        """
        Returns all cables for this rod [(origin point, direction)], such that origin point is on this rod.
        Direction is not normalized.
        """
        vectors = self.get_endpoint_a().get_cables_states(state)
        vectors.extend(self.get_endpoint_b().get_cables_states(state))
        return vectors

    # Physicsss
    def add_force_source(self, force_source):
        self._force_sources.append(force_source)
    def get_force_sources(self):
        return self._force_sources
    def get_mass(self):
        return self._mass
    def get_length(self):
        return self._length
    def get_inertia(self):
        return self._inertia 
    def get_state(self):
        return self._state  
    def get_force_torque(self, state=None):
        """
        Returns current force and torque.
        """
        if state is None:
            state = self.get_state()

        tot_force  = np.zeros(shape=(3,))
        tot_torque = np.zeros(shape=(3,))
        for source in self.get_force_sources():
            pos, force = source.get_force()
            tot_force  += force
            tot_torque += np.cross(pos, force)

        # TODO: Delete me somewhere in the not so distant future...
        tot_force  += -state.dr * self._viscosity_dr
        tot_torque += -state.w  * self._viscosity_w

        return tot_force, tot_torque

    def get_dynamics(self, state=None):
        """
        Returns current dynamics of the system (Newton-Euler)
        """
        if state is None:
            state = self.get_state()

        force, torque = self.get_force_torque(state)

        # Linear acceleration
        ddr           = force / self.get_mass()

        # Angular acceleration
        dw = np.linalg.solve(self.get_inertia(), (torque - np.cross(state.w, self.get_inertia().dot(state.w))))
        dw = dw.reshape((3,))

        return ddr, dw

    def get_kinetic_energy(self):
        state = self.get_state()
        E = state.dr.dot(state.dr) * self.get_mass() + state.w.dot(self.get_inertia().dot(state.w))

        return E

    # Physics update
    def set_state(self, state):
        self._state = state

    def update_taylor(self, state=None, dt=0.001):
        if state is None:
            state = deepcopy(self.get_state())

        ddr, dw  = self.get_dynamics(state)
        state.dr = state.dr + ddr * dt
        state.r  = state.r + state.dr * dt + 0.5*ddr*np.power(dt, 2)
        state.w  = state.w + dw*dt

        # Calculate new orientation
        v      = state.w * dt + 0.5*dw*np.power(dt, 2)
        angle  = np.linalg.norm(v)
        if not np.isclose(angle, 0.0):
            axis    = v / angle
            ro      = Rotation.from_rotvec(axis * angle)
            state.q = ro * (state.q * ro.inv())

        return state

class EndPoint:
    def __init__(self, holder):
        self._holder              = holder
        self._cabels: List[Cable] = []
        self._rotor               = None

    def add_cable(self, cable):
        self._cabels.append(cable)

    # Topological properties
    def get_cables(self):
        return copy(self._cabels)
    def get_rod(self):
        return self._holder
    def get_opposite_endpoint(self):
        if self.is_a():
            return self.get_rod().get_endpoint_b()
        else:
            return self.get_rod().get_endpoint_a()
    def is_a(self):
        return self.get_rod().get_endpoint_a() is self

    def get_position_relative_to_CoM(self, state=None):
        if state is None:
            state = self.get_rod().get_state()
        # Given position, orientation, and length -- compute the edges positions
        displacement = self.get_rod().get_length() / 2
        if self.is_a():
            displacement *= -1
        return state.q.apply([displacement, 0, 0])
    def get_vector_from_opposite_endpoint_to_me(self):
        return self.get_vector_from_endpoint_to_me(self.get_opposite_endpoint())
    def get_vector_from_endpoint_to_me(self, endpoint):
        return self.get_position() - endpoint.get_position()

    def get_position(self, state=None):
        if state is None:
            state = self.get_rod().get_state()
        return state.r + self.get_position_relative_to_CoM(state)

    def get_velocity(self, state=None):
        if state is None:
            state = self.get_rod().get_state()

        return state.dr + np.cross(self.get_position(state) - state.r, state.w)
    def get_cables_states(self, state=None):
        """
        Returns all cables for this EndPoint [(origin point, direction)], such that origin point is on this EndPoint.
        Direction is not normalized. 
        """
        if state is None:
            state = self.get_rod().get_state()

        vectors  = []
        src_pos  = self.get_position(state)
        velocity = self.get_velocity(state)
        for cable in self._cabels:
            other_endpoint = cable.get_endpoint(0)
            if other_endpoint is self:
                other_endpoint = cable.get_endpoint(1)
            other_velocity = other_endpoint.get_velocity()

            vectors.append((cable, src_pos, other_endpoint.get_position() - src_pos, velocity, other_velocity))

        return vectors

    # Force source
    def get_force(self):
        state = self.get_rod().get_state()
        cable_states = self.get_cables_states(state)
        tot_force = np.zeros(shape=(3,))
        for (cable, src_pos, cable_vector, _, _) in cable_states:
            l   = np.linalg.norm(cable_vector) - cable.get_unstretched_length()
            if l > 0:
                f_p = cable.get_stiffness() * l * (cable_vector / np.linalg.norm(cable_vector))
                tot_force += f_p

        return src_pos, tot_force

class Cable:
    def __init__(self, end_point1, end_point2, stiffness, unstretched_length, viscosity):
        self._end_points         = [end_point1, end_point2]
        self._stiffness          = stiffness
        self._unstretched_length = unstretched_length
        self._viscosity          = viscosity

        end_point1.add_cable(self)
        end_point2.add_cable(self)

    # Topological properties
    def get_endpoint(self, ind=0):
        if ind >= len(self._end_points) or ind < 0:
            raise Exception("There is no endpoint with provided index.")
        return self._end_points[ind]
    
    # Physical properties
    def get_stiffness(self):
        return self._stiffness
    def get_unstretched_length(self):
        return self._unstretched_length
    def get_viscosity(self):
        return self._viscosity
    def get_potential_energy(self):
        d = self._end_points[0].get_position() - self._end_points[1].get_position()
        P = (np.linalg.norm(d) - self.get_unstretched_length())**2 * self.get_stiffness()
        return P