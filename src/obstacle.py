# Christopher Iliffe Sprague
# sprague@kth.se

import numpy as np, matplotlib.pyplot as plt
from . import util

class Obstacle(object):

    def __init__(self, x, y, dlb, dub, nvert):

        # cartesian position
        self.p = np.array([x, y], float)

        # radius bounds
        self.rlb, self.rub = dlb/2, dub/2

        # number of vertices
        self.n = nvert

        # generate verticies
        self.gen_verts()

    def gen_verts(self):

        # angles
        thetas = np.linspace(0, 2*np.pi, self.n) + np.random.uniform(0, 2*np.pi)

        # vertices
        self.verts = np.vstack(([
            # random radius
            np.random.uniform(self.rlb, self.rub)*
            # x and y from theta
            np.array([np.cos(theta), np.sin(theta)]) +
            # position wrt origin
            self.p
            for theta in thetas
        ]))

    def point_inside(self, p):

        # extract point
        x, y = p

        # first vertex
        x0, y0 = self.verts[0, 0], self.verts[0, 1]

        inside = False
        for i in range(self.n + 1):
            x1, y1 = self.verts[i%self.n, 0], self.verts[i%self.n, 1]
            if y > min(y0, y1):
                if y <= max(y0, y1):
                    if x <= max(x0, x1):
                        if y0 != y1:
                            xints = (y-y0)*(x1-x0)/(y1-y0)+x0
                        if x0 == x1 or x <= xints:
                            inside = not inside
            x0, y0 = x1, y1

        return inside

    def line_intersect(self, seg0):

        # for each edge
        for i in range(self.n - 1):

            # edge
            seg1 = self.verts[i:i+2, :]

            # if there is an intersection
            if util.intersection(seg0, seg1):
                return True

        return False

    def plot(self, ax=None, label=False):

        if ax is None:
            fig, ax = plt.subplots(1)

        if label:
            ax.fill(self.verts[:,0], self.verts[:,1], 'gray', alpha=0.6, ec='k', label='Obstacle')
        else:
            ax.fill(self.verts[:,0], self.verts[:,1], 'gray', alpha=0.6, ec='k')

        try:
            return fig, ax
        except:
            pass


if __name__ == '__main__':

    # instantiate obstacle
    obs = Obstacle(0, 0, 10, 20, 20)

    # plot obstacle
    fig, ax = obs.plot()

    # random lines
    for i in range(10):

        line = np.array([[-10, np.random.uniform(-10, 10)], [10, 0]])
        ax.plot(line[:,0], line[:,1], 'k-')
        ax.plot(*obs.line_intersect(line), 'k.')

    plt.show()
