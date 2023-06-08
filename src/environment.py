# Christopher Iliffe Sprague
# christopher.iliffe.sprague@gmail.com

import numpy as np, matplotlib.pyplot as plt
from .obstacle import Obstacle
from . import util

class Environment(object):

    def __init__(self, lx, ly, d, n):

        # area dimensions
        self.lx = float(lx)
        self.ly = float(ly)

        # borders
        self.perim = np.array([
            [0, 0],
            [self.lx, 0],
            [self.lx, self.ly],
            [0, self.ly],
            [0, 0]
        ], float)

        # safe radius
        self.d = float(d)

        # random origin and target
        y0 = np.random.uniform(0 + self.d, self.ly - self.d)
        if y0 <= self.ly/2:
            y1 = (y0 + self.ly)/2
        elif y0 > self.ly/2:
            y1 = y0/2
        self.p0 = np.array([0.1, y0], float)
        self.pf = np.array([self.lx, y1] ,float)

        # number of obstacles
        self.nobs = n

        # generate obstacles
        self.gen_obs()

    def gen_obs(self):

        # obstacle diameter bounds
        dlb, dub = self.d, self.d*4

        # define obstacle area
        xl, xu = self.lx/10 + dub, 9*self.lx/10 - dub
        yl, yu = 0, self.ly

        # obstacle list
        self.obs = list()

        # unsucessfull tries
        i, j = 0, 0
        # first obstacle
        first = True
        # diameter bounds
        dlb, dub = self.d*2, self.d*6
        # number of vertices per obstacle
        n = 10

        while i < self.nobs  and j < 10000:

            # random position
            x = np.random.uniform(xl, xu)
            y = np.random.uniform(yl, yu)

            # proposed obstacle
            pob = Obstacle(x, y, dlb, dub, n)

            # if first
            if first:
                self.obs.append(pob)
                first = False
                i += 1

            else:

                # check conflictions with other obstacles
                if any([np.linalg.norm(pob.p - sob.p) <= pob.rub + sob.rub + self.d for sob in self.obs]):
                    j += 1
                # check if within boundaries
                elif any(pob.verts[:,0] < 0) or any(pob.verts[:,0] > self.lx) or any(pob.verts[:,1] < 0) or any(pob.verts[:,1] > self.ly):
                    j += 1
                else:
                    self.obs.append(pob)
                    i += 1
                    j = 0

    def safe(self, pts):

        # if given points
        if pts.ndim == 1:

            # check if within boundaries
            if pts[0] < 0 or pts[0] > self.lx or pts[1] < 0 or pts[1] > self.ly:
                return False

            # check if inside obstacles
            for ob in self.obs:

                if ob.point_inside(pts):
                    return False

        # if given segment
        if pts.shape == (2, 2):

            # check if intersecting boundaries
            for i in range(4):

                # boundary line
                perim = self.perim[i:i+2, :]

                if util.intersection(pts, perim):
                    return False

            # check if it intersect obstacle edges
            for ob in self.obs:

                if ob.line_intersect(pts):
                    return False

        return True


    def plot(self, ax=None, voronoi=False):

        if ax is None:
            fig, ax = plt.subplots(1)

        # plot walls
        wall = np.array([
            [0, 0],
            [0, self.ly],
            [self.lx, self.ly],
            [self.lx, 0],
            [0, 0]
        ])
        ax.plot(wall[:, 0], wall[:, 1], 'k-', label='Boundaries')
        ax.set_aspect('equal')

        # plot origin and target
        ax.plot(*self.p0, 'ks', label='Origin')
        ax.plot(*self.pf, 'kx', label='Target')

        # plot obstacles
        for i in range(len(self.obs)):
            if i == 0:
                self.obs[i].plot(ax=ax, label=True)
            else:
                self.obs[i].plot(ax=ax)

        ax.set_xlabel(r'$x \: [m]$')
        ax.set_ylabel(r'$y \: [m]$')
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=4, fancybox=True)

        try:
            return fig, ax
        except:
            pass

if __name__ == '__main__':

    env = Environment(50, 30, 1)

    fig, ax = env.plot()

    for i in range(40):

        line = np.hstack((
            np.random.uniform(-5, 55, (2, 1)),
            np.random.uniform(-5, 35, (2, 1))
        ))

        res = env.safe(line)

        if res is not True:
            ax.plot(*res, 'kx')
            ax.plot(line[:,0], line[:,1],'k-')

    plt.show()
