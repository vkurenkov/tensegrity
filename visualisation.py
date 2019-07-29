import matplotlib.pyplot as plt
import numpy             as np

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from copy                 import deepcopy

def plot_cur_state(robot, state=None):
    """
    Plots current state of the robot.
    You can pass states to be set for all the rods within the robot.
    E.g: [None, RodState(...)] -- will keep current state of the first rod and will substitue for the second rod.
    """
    # Release previously allocated figures
    plt.close("all")

    # Save current states if we try to substitue
    # TODO: Rewrite using 'with' statement
    if state is not None:
        original_state = _save_states(robot)

    # Plot current state
    fig   = plt.figure()
    ax    = Axes3D(fig)
    ax.set_title("Visualisation of the given robot state.")

    lines = _generate_one_frame_data(robot=robot, state=state)
    for (ind, line) in enumerate(lines):
        c         = None
        linewidth = None
        if ind >= len(robot.get_rods()):
            c         = "black"
            linewidth = 0.5

        ax.plot(xs=line[0], ys=line[1], zs=line[2], c=c, linewidth=linewidth)
    plt.show()

    # Restore original states
    if state is not None:
        _pass_state(robot, original_state)
def animate_historical_states(robot, states, interval=0.001):
    # Release previously allocated figures
    plt.close("all")

    fig        = plt.figure()
    ax         = Axes3D(fig)
    ax.set_title("Animation of the provided historical states.")

    lines_data = _generate_one_frame_data(robot=robot, state=_the_same_dict_at(states, 0))
    lines      = []
    for (ind, points) in enumerate(lines_data):
        c         = None
        linewidth = None
        if ind >= len(robot.get_rods()):
            c         = "black"
            linewidth = 0.5

        lines.append(*ax.plot(xs=points[0], ys=points[1], zs=points[2], c=c, linewidth=linewidth))
    
    def update(ind, states):
        num_hist_states = len(states[next(iter(states))])
        if ind >= num_hist_states:
            return
        else:
            #print("Current frame: {}".format(ind))
            data = _generate_one_frame_data(robot=robot, state=_the_same_dict_at(states, ind))
            for (points, line) in zip(data, lines):
                line.set_data(points[0], points[1])
                line.set_3d_properties(points[2])

    _ = FuncAnimation(fig, func=update, fargs=([states]), interval=interval)
    plt.show()

def _generate_one_frame_data(robot, state=None):
    # Set up the robot with the given states
    _pass_state(robot, state)
    
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
def _the_same_dict_at(src_dict, ind):
    new_dict = {}
    for key, value in src_dict.items():
        new_dict[key] = value[ind]
    return new_dict
def _save_states(robot):
    return deepcopy(robot.get_state())
def _pass_state(robot, state=None):
    if state is not None:
        robot.set_state(state)