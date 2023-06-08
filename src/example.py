# Christopher Iliffe Sprague
# sprague@kth.se

from .mission import Mission
import numpy as np, matplotlib.pyplot as plt
from .environment import Environment

def ex1():

    # instantiate mission
    mis = Mission()

    # step until collision or at target
    safe, done = True, False
    while safe and not done:

        # random steering angle
        u = np.random.uniform(-0.1, 0.1)

        # get new conditions
        s1, t1, safe, done = mis.step(u)

        # set the new conditions
        mis.set(s1, t1)

        # record conditions
        mis.record(s1, t1, u)

    # visualise trajectory
    mis.plot_traj()
    mis.plot_records()
    plt.show()

def ex2():

    # instanite mission
    mis = Mission()

    # step until collision or at target
    safe, done = True, False
    while safe and not done:

        # car position
        p = mis.state[:2]

        # relative position to target
        pt = mis.target - p

        # angle to target
        psi = np.arctan2(pt[1], pt[0])

        # car heading angle
        theta = mis.state[2]

        # steering angle towards target
        u = psi - theta

        # get new conditions
        s1, t1, safe, done = mis.step(u)

        # set new conditions
        mis.set(s1, t1)

        # record conditions
        mis.record(s1, t1, u)

    # visualise trajectory
    mis.plot_traj()
    mis.plot_records()
    plt.show()

def ex3():

    # instantiate mission
    mis = Mission()

    # control function
    u = lambda t, s: np.random.uniform(-0.1, 0.1)

    # step of some time
    states, times, controls, safe, done = mis.step(u, 200)
    mis.record(states, times, controls)

    # visualise
    mis.plot_traj()
    mis.plot_records()
    plt.show()




if __name__ == '__main__':
    #control_function_example()
    #control_sequence_example()
    #ex1()
    #ex2()
    ex3()
