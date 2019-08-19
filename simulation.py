import numpy         as np

from model                   import Rod, RodState, Cable
from control                 import controller

from tqdm                    import tqdm
from collections             import defaultdict

def run_simulation(robot, time, dt=0.001):
    n_iterations = int(time / dt)
    hist_states  = defaultdict(list)
    for _ in tqdm(range(n_iterations)):
        # Update the robot
        ## controller(rotors)
        robot.update(dt=dt)

        # Collect updated data
        rod_states = robot.get_state()
        for key in rod_states.keys():
            hist_states[key].append(rod_states[key])

    return hist_states