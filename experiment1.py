import visualisation          as rob_vis
import configurations.n_prism as n_prism
import matplotlib.pyplot      as plt
import matplotlib             as mplt
import numpy                  as np

from simulation          import  run_simulation

# Build a robot
config = n_prism.DEFAULT_CONFIG.copy()
config["Rod_Mass"] = 1.0
config["Rod_Length"] = 0.3 * 3
config["Cable_Stiffness"] = 10
config["Cable_Viscosity"] = 1
config["CableV_UnstretchedLength"] = 0.2
config["CableH1_UnstretchedLength"] = 0.2
config["CableH2_UnstretchedLength"] = 0.2
robot = n_prism.build(n=3, config=config)

# Enable gravity
#robot.enable_gravity()

# Run simulation
total_time = 5.0
dt         = 0.005
hist_states, states = run_simulation(robot, time=total_time, dt=dt)

# Show simulation
# rob_vis.animate_historical_states(robot, hist_states)

# Show some key frames
rob_vis.plot_cur_state(robot, states[0], time=0.0)
rob_vis.plot_cur_state(robot, states[int(0.2 / dt)], time=0.2)
rob_vis.plot_cur_state(robot, states[int(0.8 / dt)], time=0.8)
rob_vis.plot_cur_state(robot, states[int(1.6 / dt)], time=1.6)
rob_vis.plot_cur_state(robot, states[int(3.2 / dt)], time=3.2)
rob_vis.plot_cur_state(robot, states[int(4.9 / dt)], time=5.0)

# Set up the style
a = mplt.rcParams.keys()
mplt.rcParams.update({
    'font.size': 12, 
    "font.family": "Tibetan Machine Uni", 
    "lines.linewidth": 0.5,
    "axes.grid": True
})

# Draw Center of Mass of the robot
fig, ax = plt.subplots(nrows=3, ncols=1)
ax[0].plot(np.arange(0.0, total_time, dt), [pos[0] for pos in hist_states["CoM"]], )
ax[0].set_xlabel("t (in seconds)")
ax[0].set_ylabel("X")

ax[1].plot(np.arange(0.0, total_time, dt), [pos[1] for pos in hist_states["CoM"]])
ax[1].set_xlabel("t (in seconds)")
ax[1].set_ylabel("Y")

ax[2].plot(np.arange(0.0, total_time, dt), [pos[2] for pos in hist_states["CoM"]])
ax[2].set_xlabel("t (in seconds)")
ax[2].set_ylabel("Z")

fig.suptitle("Center of Mass over Time")
plt.subplots_adjust(hspace=0.4)
plt.show()

# Draw potential energy in every cable
fig, ax = plt.subplots(nrows=3, ncols=3)
potential_energies = [hist_states[key] for key in hist_states if "Potential" in key]
ind = 0
for row in ax:
    for col in row:
        col.plot(np.arange(0.0, total_time, dt), potential_energies[ind])
        col.set_xlabel("t (in seconds)")
        col.set_ylabel("Cable {}".format(ind))
        ind += 1
fig.suptitle("Potential Energy over Time")
plt.subplots_adjust(hspace=0.4)
plt.subplots_adjust(wspace=0.4)
plt.show()

# Relative cable length change
fig, ax = plt.subplots(nrows=3, ncols=3)
zero_time  = [hist_states[key][0] for key in hist_states if "UnstretchedLength" in key]
rel_change = [hist_states[key][1:] for key in hist_states if "_Length" in key]
ind = 0
for row in ax:
    for col in row:
        col.plot(np.arange(dt, total_time, dt), np.array(rel_change[ind]) - zero_time[ind])
        print("Cable {}".format(ind))
        print((np.array(rel_change[ind]) - zero_time[ind])[-1])
        col.set_xlabel("t (in seconds)")
        col.set_ylabel("Cable {}".format(ind))
        ind += 1
fig.suptitle("Relative Length Change over Time")
plt.subplots_adjust(hspace=0.4)
plt.subplots_adjust(wspace=0.4)
plt.show()