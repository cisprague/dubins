# Christopher Iliffe Sprague
# sprague@kth.se

import numpy as np, matplotlib.pyplot as plt
from scipy.integrate import RK45 as ODE
from .dynamics import Dynamics
from .environment import Environment
np.set_printoptions(suppress=True, precision=4)

class Mission(object):

    def __init__(self, dyn=None, env=None):

        if dyn is None:
            dyn = Dynamics(1, 1)
        if env is None:
            env = Environment(50, 30, 1, 20)

        # dynamics
        self._dynamics = dyn

        # environment
        self._environment = env

        # origin and target
        self.origin = self._environment.p0
        self.target = self._environment.pf

        # boundaries
        self.xbound = np.array([0, self._environment.lx])
        self.ybound = np.array([0, self._environment.ly])

        # arbitrary control for integrator initialisation
        self._control = 0

        # numerical integrator
        self._integrator = ODE(
            self._eom,
            0,
            np.hstack((self.origin, np.zeros(self._dynamics.sdim - 2))),
            1000,
            atol=1e-8,
            vectorized=True,
            max_step=0.05,
            #jac=self._jac
        )

        # reset mission
        self.reset()

    def set(self, state, time):

        # set state and time for user
        self.state = np.array(state, float)
        self.time  = float(time)

        # set numerical integrator state and time
        self._integrator.y = np.array(self.state, float)
        self._integrator.t = float(self.time)

    def reset(self):

        # initial state and time
        s0 = np.hstack((self.origin, np.zeros(self._dynamics.sdim - 2)))
        t0 = 0

        # set to nominal conditions
        self.set(s0, t0)

        # reset records
        self.states   = np.array([s0], float)
        self.times    = np.array([t0], float)
        self.controls = np.empty((1, 0), float)

    def record(self, state, control, time):

        # record state, time, and control
        self.states   = np.vstack((self.states, state))
        self.times    = np.append(self.times, time)
        self.controls = np.append(self.controls, control)

    def _eom(self, t, state):
        return self._dynamics.eom_state(state, self._control)

    def _jac(self, t, state):
        return self._dynamics.eom_state_jac(state, self._control)

    def safe(self, p0, p1=None):

        p0 = np.array(p0, float)

        # if only given one position
        if p1 is None:
            return self._environment.safe(p0)
        # if given a line
        else:
            p1 = np.array(p1)
            return self._environment.safe(np.vstack((p0, p1)))

    def done(self, p):
        p = np.array(p, float)
        delta = np.linalg.norm(p - self.target)
        if delta < 0.1:
            return True
        else:
            return False

    def step(self, control, Dt=None, inplace=False, verbose=False, record=False):

        # store the starting state, time, and direction
        s0, t0 = self._integrator.y, self._integrator.t

        # if given a desire boundary time
        if isinstance(Dt, float) or isinstance(Dt, int):

            # final time
            tf = t0 + Dt

            # create records
            states   = np.empty((0,3), float)
            times    = np.empty((1,0), float)
            controls = np.empty((1,0), float)

            # integrate until desired time is reached
            safe, done = True, False

            # if integrating forward
            while safe and not done:

                # if control is a function
                if callable(control):
                    self._control = control(self._integrator.t, self._integrator.y)
                else:
                    self._control = control

                # integration step
                self._integrator.step()

                # extract new state and time
                s1, t1 = self._integrator.y, self._integrator.t

                # adjust if over boundry
                if self._integrator.direction == 1 and t1 >= tf:
                    t1 = tf
                    s1 = self._integrator.dense_output()(t1)
                    final = True
                else:
                    final = False

                # check safety of new position
                if not self.safe(s1[:2]):
                    safe = False
                # check for intersection
                elif not self.safe(s0[:2], s1[:2]):
                    safe = False
                # if we good
                else:
                    safe = True

                # check if near target
                done = self.done(s1[:2])

                # print if desired
                if verbose:
                    print('State: {0:<30} Time: {1:<10.3f} Control: {2:<10.3f} Safe: {3:<10} Done: {4:<10}'.format(str(s1), t1, self._control, str(safe), str(done)))

                # records
                states   = np.vstack((states, s1))
                times    = np.append(times, t1)
                controls = np.append(controls, self._control)

                # break if final
                if final:
                    break
                else:
                    continue

            s, u, t = states, controls, times

        # if just wanting one step
        elif Dt is None:

            # if given control function
            if callable(control):
                self._control = control(self._integrator.t, self._integrator.y)
            else:
                self._control = control

            # take one integration step
            self._integrator.step()

            # extract new state and time
            s1, t1 = self._integrator.y, self._integrator.t

            # check safety of new position
            if not self.safe(s1[:2]):
                safe = False
            # check for intersection
            elif not self.safe(s0[:2], s1[:2]):
                safe = False
            # if we good
            else:
                safe = True

            # check if near target
            done = self.done(s1[:2])

            # print if desired
            if verbose:
                print('State: {0:<30} Time: {1:<10.3f} Control: {2:<10.3f} Safe: {3:<10} Done: {4:<10}'.format(str(s1), t1, self._control, str(safe), str(done)))

            s, t, u = s1, t1, self._control

        if inplace:
            self.set(s1, t1)
        elif not inplace:
            self.set(s0, t0)
        if record:
            self.record(s, u, t)

        return s, u, t, safe, done

    def simulate(self, controls, times, verbose=False):

        # reset state and time record
        self.reset()

        for i in range(len(controls)):

            # integrate
            s, u, t, safe, done = self.step(controls[i], Dt=times[i+1]-self.time, inplace=True, verbose=verbose, record=True)

            if not safe or done:
                break

        # if succesful
        if safe and done:
            return 1

        # if unsuccesful
        else:

            # origin distance from target
            D = np.linalg.norm(self.target - self.origin)

            # car distance to target
            d = np.linalg.norm(self.target - s[-1,:2])

            # return percent distance acheived
            return 1 - d/D

    def plot_traj(self, ax=None):

        if ax is None:
            fig, ax = plt.subplots(1)

        # plot trajectory
        ax.plot(self.states[:,0], self.states[:,1], 'k-')

        # plot environment
        try:
            if ax.env == True:
                pass
        except:
            self._environment.plot(ax)
            ax.env = True

        try:
            return fig, ax
        except:
            pass

    def plot_records(self, ax=None):

        if ax is None:
            fig, ax = plt.subplots(self._dynamics.sdim + 1, sharex=True)

        # plot states
        for i in range(self._dynamics.sdim):
            ax[i].plot(self.times, self.states[:,i], 'k-')
            ax[i].set_ylabel(self._dynamics.syms[i])

        # plot controls
        try:
            ax[-1].plot(self.times[:-1], self.controls, 'k-')
            ax[-1].set_ylabel(r'$\phi \: [rad]$')
            ax[-1].set_xlabel(r'$t \: [s]$')
        except:
            pass

        try:
            return fig, ax
        except:
            pass

if __name__ == '__main__':

    # instantiate mission
    mis = mission()

    # control function
    uf = lambda t, s: np.random.uniform(-0.3, 0.3)

    # step for 5 seconds
    states, controls, times, safe, done = mis.step(uf)

    print(states)
