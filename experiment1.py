import visualisation          as rob_vis
import configurations.n_prism as n_prism

from simulation          import  run_simulation

# Build a robot
config = n_prism.DEFAULT_CONFIG.copy()
config["Rod_Mass"] = 1.0
config["Rod_Length"] = 0.3
config["Cable_Stiffness"] = 10
config["Cable_Viscosity"] = 1
config["CableV_UnstretchedLength"] = 0.2
config["CableH1_UnstretchedLength"] = 0.2
config["CableH2_UnstretchedLength"] = 0.2
robot = n_prism.build(n=3)

# Enable gravity
#robot.enable_gravity()

# Plot current state, simulate, and plot the last state
rob_vis.plot_cur_state(robot)
hist_states = run_simulation(robot, time=3, dt=0.005)
rob_vis.plot_pot_energy_over_time(hist_states)
rob_vis.plot_cur_state(robot)
rob_vis.plot_com_graphs(hist_states)

