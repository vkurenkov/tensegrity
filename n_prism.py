import visualisation          as rob_vis
import configurations.n_prism as n_prism

from model          import  run_simulation

# Build a robot
robot = n_prism.build(n=7)

# Enable gravity
#robot.enable_gravity()

# Plot current state, simulate, and plot the last state
rob_vis.plot_cur_state(robot)
hist_states = run_simulation(robot, time=3, dt=0.005)
rob_vis.plot_cur_state(robot)
rob_vis.plot_com_graphs(hist_states)

