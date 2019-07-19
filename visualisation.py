import matplotlib.pyplot as plt
import numpy             as np

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from copy                 import deepcopy

class TensegrityRobotVisualiser:
    def plot_cur_state(self, robot, states=None):
        """
        Plots current state of the robot.
        You can pass states to be set for all the rods within the robot.
        E.g: [None, RodState(...)] -- will keep current state of the first rod and will substitue for the second rod.
        """
        # Release previously allocated figures
        plt.close("all")

        # Save current states if we try to substitue
        # TODO: Rewrite using 'with' statement
        if states is not None:
            original_states = self._save_states(robot)

        # Plot current state
        fig   = plt.figure()
        ax    = Axes3D(fig)
        ax.set_title("Visualisation of the given robot state.")

        lines = self._generate_one_frame_data(robot=robot, states=states)
        for (ind, line) in enumerate(lines):
            c         = None
            linewidth = None
            if ind >= len(robot.get_rods()):
                c         = "black"
                linewidth = 3

            ax.plot(xs=line[0], ys=line[1], zs=line[2], c=c, linewidth=linewidth)
        plt.show()

        # Restore original states
        if states is not None:
            self._pass_states(robot, original_states)
    def animate_historical_states(self, robot, states, interval=0.001):
        # Release previously allocated figures
        plt.close("all")

        fig        = plt.figure()
        ax         = Axes3D(fig)
        ax.set_title("Animation of the provided historical states.")

        lines_data = self._generate_one_frame_data(robot=robot, states=states[0])
        lines      = []
        for (ind, points) in enumerate(lines_data):
            c         = None
            linewidth = None
            if ind >= len(robot.get_rods()):
                c         = "black"
                linewidth = 3

            lines.append(*ax.plot(xs=points[0], ys=points[1], zs=points[2], c=c, linewidth=linewidth))
        
        def update(ind, states):
            if ind >= len(states):
                return
            else:
                data = self._generate_one_frame_data(robot=robot, states=states[ind])
                for (points, line) in zip(data, lines):
                    line.set_data(points[0], points[1])
                    line.set_3d_properties(points[2])

        _ = FuncAnimation(fig, func=update, fargs=([states]), interval=interval)
        plt.show()
    def _generate_one_frame_data(self, robot, states=None):
        # Set up the robot with the given states
        self._pass_states(robot, states)
        
        # Draw rods
        lines = []
        for rod in robot.get_rods():
            pos1 = rod.get_endpoint_a().get_position()
            pos2 = rod.get_endpoint_b().get_position()

            lines.append(([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]]))

        # Draw cables
        for cable in robot.get_cables():
            pos1 = cable.get_endpoint(0).get_position()
            pos2 = cable.get_endpoint(1).get_position()

            lines.append(([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]]))
        return lines

    def _save_states(self, robot):
        states = []
        for rod in robot.get_rods():
            states.append(deepcopy(rod.get_state()))
        return states
    def _pass_states(self, robot, states=None):
        if states is not None:
            for rod, state in zip(robot.get_rods(), states):
                if state is None:
                    continue
                else:
                    rod.set_state(state)
        else:
            return   