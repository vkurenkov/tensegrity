import numpy as np

from typing                  import List, Tuple, Iterable
from scipy.spatial.transform import Rotation
from dataclasses             import dataclass
from copy                    import copy
from collections             import OrderedDict

class TensegrityRobot:
    def __init__(self):
        self._rods   = []
        self._cables = []

    def update(self, dt=0.1):
        """
        Updates all rods.
        """
        for rod in self.get_rods():
            rod.update(dt=dt)

    def add_rods(self, rods):
        self._rods.extend(rods)
    def add_cables(self, cables):
        self._cables.extend(cables)
    def get_rods(self):
        return self._rods
    def get_cables(self):
        return self._cables

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
    def __init__(self, mass, length, inertia, state):
        self._mass       = mass
        self._length     = length
        self._inertia    = inertia
        self._state      = state
        self._endpoint_a = EndPoint(holder=self)
        self._endpoint_b = EndPoint(holder=self)

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

    # Physical properties
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

        cable_vectors = self.get_cables_states(state)
        force         = np.zeros(3,)
        torque        = np.zeros(3,)
        for (cable, src_pos, direction, out_velocity, in_velocity) in cable_vectors:
            l   = np.linalg.norm(direction) - cable.get_unstretched_length()
            v   = in_velocity - out_velocity
            
            f_d = cable.get_viscosity() * v
            f_p = cable.get_stiffness() * l * (direction / np.linalg.norm(direction))
            f   = f_d + f_p
            tau = np.cross(src_pos, f)

            force  += f
            torque += tau

        return force, torque
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
        dw = np.linalg.solve(self.get_inertia(), (torque - np.cross(state.w, self.get_inertia() * state.w)))
        dw = dw.reshape((3,))

        return ddr, dw

    # Physics update
    def set_state(self, state):
        self._state = state
    def update(self, state=None, dt=0.1):
        """
        Update the state of the rod.
        """
        if state is None:
            state = self.get_state()

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

    def add_cable(self, cable):
        self._cabels.append(cable)

    # Topological properties
    def get_cables(self):
        return copy(self._cabels)
    def get_rod(self):
        return self._holder
    def is_a(self):
        return self.get_rod().get_endpoint_a() is self

    def get_position(self, state=None):
        # Given position, orientation, and length -- compute the edges positions
        displacement = self.get_rod().get_length() / 2
        if self.is_a():
            displacement *= -1

        if state is None:
            state = self.get_rod().get_state()

        return state.r + state.q.apply([displacement, 0, 0])
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