import matplotlib.pyplot as plt
import numpy             as np

from mpl_toolkits.mplot3d import Axes3D
from copy                 import deepcopy

def plot(robot):
    fig   = plt.figure()
    ax    = Axes3D(fig)
    ax.set_title("Visualisation of the robot state.")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    # Draw rods
    for i, rod in enumerate(robot.get_rods()):
        pos1 = rod.get_endpoint_a().get_position()
        pos2 = rod.get_endpoint_b().get_position()

        ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]], linewidth=3, c="red")
        ax.text(pos2[0] + (pos1[0] - pos2[0]) / 2, pos2[1] + (pos1[1] - pos2[1]) / 2, pos2[2] + (pos1[2] - pos2[2]) / 2, "rod_{}".format(i+1))

    # Draw cables
    for cable in robot.get_cables():
        pos1 = cable.get_endpoint(0).get_position()
        pos2 = cable.get_endpoint(1).get_position()

        ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], [pos1[2], pos2[2]], c="black", linewidth=0.5)
    plt.show()