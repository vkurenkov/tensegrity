import numpy as np

from typing                  import List, Tuple, Iterable
from scipy.spatial.transform import Rotation
from dataclasses             import dataclass
from copy                    import copy
from collections             import OrderedDict

def quaternion_multiply(quaternion1, quaternion0):
    x0, y0, z0, w0 = quaternion0.flatten()
    x1, y1, z1, w1 = quaternion1.flatten()
    return np.array([x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                     -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0], dtype=np.float64)


class TensegrityRobot:
    def __init__(self):
        self._rods   = OrderedDict()
        self._cables = OrderedDict()

    # Controller
    def add_rods(self, rods):
        for rod in rods:
            if id(rod) in self._rods:
                raise Exception("One of the rods is already in the robot.")
            else:
                self._rods[id(rod)] = rod
    def add_cables(self, cables):
        for cable in cables:
            end_point0 = cable.get_endpoint(0)
            end_point1 = cable.get_endpoint(1)
            cable_id   = frozenset([id(end_point0), id(end_point1)])
            if cable_id in self._cables:
                raise Exception("Cable with such endpoints is already presented. (Note that the order of endpoints does not matter)")

            rod1 = end_point0.get_rod()
            rod2 = end_point1.get_rod()
            if id(rod1) not in self._rods or id(rod2) not in self._rods:
                raise Exception("Cable connects one or more unknown rods.")

            # In order to keep the topology of robot correct
            # We assign cables to endpoints here
            # Thus all links are correct only after everything was added to the objects of this class
            end_point0._cabels.append(cable)
            end_point1._cabels.append(cable)
            self._cables[cable_id] = cable
                
    def update(self, dt=0.1):
        """
        Updates all rods.
        """
        for rod in self.get_rods():
            rod.update(dt=dt)

    # Getters
    def get_rods(self):
        return self._rods.values()
    def get_cables(self):
        return self._cables.values()

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
    def get_cables_vectors(self, state=None):
        """
        Returns all cables for this rod [(origin point, direction)], such that origin point is on this rod.
        Direction is not normalized.
        """
        vectors = self.get_endpoint_a().get_cables_vectors(state)
        vectors.extend(self.get_endpoint_b().get_cables_vectors(state))
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

        cable_vectors = self.get_cables_vectors(state)
        force         = np.zeros(3,)
        torque        = np.zeros(3,)
        for (cable, src_pos, direction) in cable_vectors:
            l   = np.linalg.norm(direction) - cable.get_unstretched_length()
            f   = cable.get_stiffness() * l * direction
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
        v = state.w * dt + 0.5*dw*np.power(dt, 2)
        angle  = np.linalg.norm(v)
        if angle == 0.0:
            angle = 1
        axis   = v / angle

        cur_ro     = state.q.as_quat()
        new_ro     = Rotation.from_rotvec(axis * angle)
        new_ro_inv = new_ro.inv().as_quat()
        new_ro     = new_ro.as_quat()

        state.q = Rotation.from_quat(quaternion_multiply(new_ro, quaternion_multiply(cur_ro, new_ro_inv)))
        return state

class EndPoint:
    def __init__(self, holder):
        self._holder              = holder
        self._cabels: List[Cable] = []

    # Topological properties
    def get_cables(self):
        return copy(self._cabels)
    def get_rod(self):
        return self._holder
    def is_a(self):
        return self.get_rod().get_endpoint_a() is self

    # Geometrical properties
    def get_position(self, state=None):
        # Given position, orientation, and length -- compute the edges positions
        displacement = self.get_rod().get_length() / 2
        if self.is_a():
            displacement *= -1

        if state is None:
            state = self.get_rod().get_state()

        return state.r + state.q.apply([displacement, 0, 0])
    def get_cables_vectors(self, state=None):
        """
        Returns all cables for this EndPoint [(origin point, direction)], such that origin point is on this EndPoint.
        Direction is not normalized. 
        """
        if state is None:
            state = self.get_rod().get_state()

        vectors = []
        src_pos = self.get_position(state)
        for cable in self._cabels:
            other_endpoint = cable.get_endpoint(0)
            if other_endpoint is self:
                other_endpoint = cable.get_endpoint(1)

            vectors.append((cable, src_pos, other_endpoint.get_position() - src_pos))

        return vectors

class Cable:
    def __init__(self, end_point1, end_point2, stiffness, unstretched_length):
        if end_point1.get_rod() is end_point2.get_rod():
            raise Exception("End points must be on different rods.")

        self._end_points         = [end_point1, end_point2]
        self._stiffness          = stiffness
        self._unstretched_length = unstretched_length

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
