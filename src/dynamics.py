# Christopher Iliffe Sprague
# sprague@kth.se

import numpy as np

class Dynamics(object):

    '''
    Dubin's car model with constant speed, where the control parameter,
    u ∈ [-1, 1], determines the steering angle.
    '''

    def __init__(self, length, speed):

        # car length [m]
        self.l = float(length)
        self.v = float(speed)

        # state and control dimensions
        self.sdim = 3
        self.udim = 1

        # state symbols
        self.syms = [r'$x \: [m]$', r'$y \: [m]$', r'$\theta \: [rad]$']

    def eom_state(self, state, control):

        # extract state
        x, y, theta = state

        # extract control
        phi = control

        # return state transition
        return np.array([
            self.v*np.cos(theta),
            self.v*np.sin(theta),
            np.tan(phi)/self.l
        ], float)

    def eom_state_jac(self, state, control):

        # extract state
        x, y, theta = state

        # extract control
        u = control

        # return state transition Jacobian
        return np.array([
            [0, 0, -np.sin(theta)],
            [0, 0,  np.cos(theta)],
            [0, 0,              0]
        ], float)

class Dynamics_2nd(object):

    '''
    Dubin's car model with constant speed, where the control parameter,
    u ∈ [-1, 1], determines the acceleration of the steering angle.
    This enforces smoothness in the steering angle.
    Functions for Pontryagin's maximum principle are also included.
    '''

    def __init__(self, length, alpha=0):

        # car length
        self.l = float(length)

        # homotopy parameter
        self.alpha = float(alpha)

        # state and control dimensions
        self.sdim = 5
        self.udim = 1

    def eom_state(self, state, control):

        # extract state
        x, y, theta, phi, omega = state

        # extract control
        u = control

        # return state transition
        return np.array([
            np.cos(theta),
            np.sin(theta),
            np.tan(phi)/self.l,
            omega,
            u
        ], float)

    def eom_state_jac(self, state, control):

        # extract state
        x, y, theta, phi, omega = state

        # extract control
        u = control

        # return state transition jacobian
        return np.array([
            [0, 0, -np.sin(theta), 0,                         0],
            [0, 0, np.cos(theta),  0,                         0],
            [0, 0, 0,              1/(self.l*np.cos(phi)**2), 0],
            [0, 0, 0,              0,                         1],
            [0, 0, 0,              0,                         0]
        ], float)

    def lagrangian(self, control):

        # extract control
        u = control

        # return lagrangian
        return u*(self.alpha + u*(-self.alpha + 1))

    def hamiltonian(self, fullstate, control, homotopy):

        # extract fullstate
        x, y, theta, phi, omega, lx, ly, ltheta, lphi, lomega = fullstate

        # extract control
        u = control

        # return hamiltonian
        return (ltheta*np.tan(phi) + l*(self.alpha*u + lomega*u + lphi*omega + lx*np.cos(theta) + ly*np.sin(theta) + u**2*(-self.alpha + 1)))/self.l

    def eom_fullstate(self, fullstate, control):

        # extract fullstate
        x, y, theta, phi, omega, lx, ly, ltheta, lphi, lomega = fullstate

        # extract control
        u = control

        # subexpression elimination
        x0 = np.cos(theta)
        x1 = np.sin(theta)
        x2 = 1/self.l

        # return fullstate transition
        return np.array([
            x0,
            x1,
            x2*np.tan(phi),
            omega,
            u,
            0,
            0,
            lx*x1 - ly*x0,
            -ltheta*x2/np.cos(phi)**2,
            -lphi
        ], float)

    def eom_fullstate_jac(self, fullstate, control):

        # extract fullstate
        x, y, theta, phi, omega, lx, ly, ltheta, lphi, lomega = fullstate

        # extract control
        u = control

        # common subexpression elimination
        x0 = np.sin(theta)
        x1 = np.cos(theta)
        x2 = 1/(l*np.cos(phi)**2)

        # return fullstate transition jacobian
        return np.array([
            [0, 0,           -x0,                        0, 0,  0,   0,   0,  0, 0],
            [0, 0,            x1,                        0, 0,  0,   0,   0,  0, 0],
            [0, 0,             0,                       x2, 0,  0,   0,   0,  0, 0],
            [0, 0,             0,                        0, 1,  0,   0,   0,  0, 0],
            [0, 0,             0,                        0, 0,  0,   0,   0,  0, 0],
            [0, 0,             0,                        0, 0,  0,   0,   0,  0, 0],
            [0, 0,             0,                        0, 0,  0,   0,   0,  0, 0],
            [0, 0, lx*x1 + ly*x0,                        0, 0, x0, -x1,   0,  0, 0],
            [0, 0,             0, -2*ltheta*x2*np.tan(phi), 0,  0,   0, -x2,  0, 0],
            [0, 0,             0,                        0, 0,  0,   0,   0, -1, 0]
        ], float)

    def pontryagin(self, fullstate):

        # extract fullstate
        x, y, theta, phi, omega, lx, ly, ltheta, lphi, lomega = fullstate

        # return optimal control
        return (self.alpha + lomega)/(2*(self.alpha - 1))
