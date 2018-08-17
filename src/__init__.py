# Christopher Iliffe Sprague
# sprague@kth.se

from math import cos, sin, tan, pi, atan2
from random import uniform

def evaluate(solution_function, grade, obs=None):

    # if obs is True: precomputed obstacles used
    # if obs is [(x, y, r), ... , (x, y, r)]: use that obstacle config
    # if obs is None; use random obstacles

    if obs is True:
        obs = [
        [
            10.234897348278846,
            7.426454970638634,
            0.6036968449353223
        ],
        [
            9.62360178529774,
            3.5024934390492084,
            0.640790030775506
        ],
        [
            5.545792206236376,
            3.1172535599361098,
            0.5082665691945417
        ],
        [
            12.934926733980326,
            6.731061250216145,
            0.5959879020760377
        ],
        [
            8.670001702247484,
            0.8983442377484021,
            0.652618220438745
        ],
        [
            15.07117247578524,
            5.542943578444764,
            0.5128733414030838
        ],
        [
            5.161444180972378,
            9.205934481925263,
            0.5080914934599107
        ],
        [
            13.453025054312407,
            3.4000502649247615,
            0.7522400709399546
        ],
        [
            7.6621668116269674,
            8.620628500804685,
            0.7754175898925023
        ],
        [
            5.459719078188462,
            6.211824386849173,
            0.6247372877261753
        ],
        [
            14.542538486684606,
            9.274980363181944,
            0.6507933480043415
        ],
        [
            15.171972300252216,
            1.0064424626594783,
            0.7452862888001911
        ],
        [
            4.932792992444652,
            0.883669003600472,
            0.6876386432133378
        ],
        [
            11.353164353829955,
            0.752043219264199,
            0.5315610243578464
        ],
        [
            12.18303789861043,
            8.993623583484608,
            0.6404094018062996
        ],
        [
            7.51339002828059,
            4.370588682815919,
            0.5287579124306581
        ],
        [
            11.476995854637153,
            4.941636225638853,
            0.6015407536368719
        ],
        [
            8.342699317665362,
            6.312142803490614,
            0.5550384214056028
        ]
        ]
    else:
        pass

    # initialise environment
    env = Environment(obs=obs)

    # initialise car with that environment
    car = Car(env)

    # execute student's solution function
    controls, times = solution_function(car)

    # return state, control, and time lists, along with doneness and car
    xl, yl, thetal, ul, tl, done = car.evaluate(controls, times, grade)
    return car, xl, yl, thetal, ul, tl, done


class Car(object):

    '''
    Dubin's car
    '''

    def __init__(self, env=None):

        # set environment
        if env is None:
            self._environment = Environment()
        else:
            self._environment = env

        # random initial position [m]
        self.x0 = float(0)
        self.y0 = self._environment.ly*0.2

        # random target position [m]
        self.xt = self._environment.lx
        self.yt = self._environment.ly*0.8

        # time step size [s]
        self.dt = 0.01

        # list of obstacles
        self.obs = [(ob.x, ob.y, ob.r) for ob in self._environment.obstacles]

        # environment bounds [m]
        self.xlb = float(0)
        self.xub = self._environment.lx
        self.ylb = float(0)
        self.yub = self._environment.ly

    def step(self, x, y, theta, phi):

        '''
        Returns a new state (xn, yn, thetan), safety condition,
        and done condition, given an initial state (x, y, theta)
        and control phi.
        Numerical integration is done at a time step of dt [sec].
        '''

        # state rate
        dx     = cos(theta)
        dy     = sin(theta)
        dtheta = tan(phi)

        # new state (forward Euler integration)
        xn     = x     + self.dt*dx
        yn     = y     + self.dt*dy
        thetan = theta + self.dt*dtheta

        return xn, yn, thetan

    def evaluate(self, controls, times, grade):

        '''
        Returns a simulated final result, given a sequence of controls
        and times. controls[i] is considered constant between times[i]
        and times[i+1].
        '''

        grades = ['E', 'C', 'A']

        if len(controls) != len(times) - 1:
            raise ValueError("Control sequence length must be 1 less than times.")
        elif times[0] != 0:
            raise ValueError("First time must be 0.")
        elif any([phi < -pi/4 or phi > pi/4 for phi in controls]):
            raise ValueError("All controls must be between -pi/4 and pi/4.")
        elif any([times[i] >= times[i+1] for i in range(len(controls))]):
            raise ValueError("Time sequence must be increasing.")
        elif any([times[i+1] - times[i] < self.dt-0.000001 for i in range(len(controls))]):
            raise ValueError("Difference in each time must be greater than 0.01 [s].")
        elif grade not in grades:
            raise ValueError("Evaluation grade must be in {}.".format(str(grades)))

        # states and time lists with initial configuration
        xl     = [self.x0]
        yl     = [self.y0]
        thetal = [0]
        ul     = []
        tl     = [0]

        for i in range(len(controls)):

            # simulate between times
            while tl[-1] < times[i+1]:

                # get new state and time
                xn, yn, thetan = self.step(xl[-1], yl[-1], thetal[-1], controls[i])

                # record state
                xl.append(xn)
                yl.append(yn)
                thetal.append(thetan)

                # record control
                ul.append(controls[i])

                # record time
                tl.append(tl[-1] + self.dt)

                # break if not safe
                if grade is 'E':
                    if not self._environment.inbounds(xn, yn):
                        term = True
                        break
                    else:
                        term = False
                if grade is "C" or grade is "A":
                    if not self._environment.safe(xn, yn):
                        term = True
                        break
                    else:
                        term = False

            if term == True:
                break

        # extend last control to signify constant value between times
        ul.append(ul[-1])

        # safety
        safe = self._environment.safe(xn, yn)

        # grades
        if grade is "E":
            done = True if abs(self.xt-xn) <= 0.1 else False
        elif grade is "C":
            done = True if ((self.xt-xn)**2+(self.yt-yn)**2)**0.5 <= 0.1 else False
        elif grade is "A":
            done = True if ((self.xt-xn)**2+(self.yt-yn)**2)**0.5 <= 0.1 and abs(thetan) <= 0.1 else False

        # return states, controls, times, final saftey, and done
        return xl, yl, thetal, ul, tl, done

class Environment(object):

    '''
    Environment described by a rectangle [(0,0),(Lx,0),(Lx,Ly),(0,Ly)],
    containing n circular obstacles of radius r, positioned randomly
    within the rectangle.
    '''


    def __init__(self, obs=None):

        # horizontal length
        self.lx  = float(20)
        # vertical length
        self.ly  = float(10)

        # if given obstacle list
        if obs is not None:
            self.obstacles = [Obstacle(*ob) for ob in obs]

        # otherwise initialise obstacles randomly
        else:
            self.init_obstacles()

    def init_obstacles(self):

        '''
        Initialises as many obstacles, with 1-2 [m] radius, as possible
        with at least 2 [m] clearence.
        '''

        # obstacle list
        self.obstacles = list()
        # number of unsuccesful obstacles
        first, i = True, 0
        # generate random obstacles
        while i < 20000:
            # random radius
            r = uniform(0.5, 0.8)
            # random position
            x = uniform(self.lx*0.2 + r, self.lx*0.8 - r)
            y = uniform(r, self.ly - r)
            # first obstacle
            if first:
                self.obstacles.append(Obstacle(x, y, r))
                first = False
            # subsequent obstacles
            else:
                # loop through prexisting obstacles
                for ob in self.obstacles:
                    # compute distance to obstacle
                    d = ((x-ob.x)**2 + (y-ob.y)**2)**0.5
                    # discard obstacle if not 2 [m] clearence
                    if d < ob.r + r + 1:
                        good = False
                        break
                    # keep testing if clearence is good
                    else:
                        good = True
                # if obstacle is acceptable
                if good:
                    self.obstacles.append(Obstacle(x,y,r))
                    i = 0
                else:
                    i += 1
                    continue

    def inbounds(self, x, y):

        # if past east wall
        if x < 0:
            return False
        # if past west wall
        elif x > self.lx:
            return False
        # if past south wall
        elif y < 0:
            return False
        # if past north wall
        elif y > self.ly:
            return False
        else:
            return True

    def obstacle_free(self, x, y):

        for ob in self.obstacles:
            if not ob.safe(x, y):
                return False
        return True

    def safe(self, x, y):

        ''' Tests whether a point is within boundaries and obstacle free. '''

        if self.inbounds(x, y) and self.obstacle_free(x, y):
            return True
        else:
            return False

class Obstacle(object):

    ''' Circular obstacle of radius r positioned at (x,y). '''

    def __init__(self, x, y, r):

        # position
        self.x = float(x)
        self.y = float(y)

        # radius
        self.r = float(r)

    def safe(self, x, y):

        ''' Tests whether a point is within the obstacle. '''

        # relative position
        x -= self.x
        y -= self.y

        # distance to obstacle
        d = (x**2 + y**2)**0.5

        # if intersecting obstacle
        return False if d <= self.r else True

if __name__ == "__main__":

    """
    obsl = list()
    for i in range(8):

        # initialise obstacles randomly
        env = Environment()
        obsl.append([(ob.x, ob.y, ob.r) for ob in env.obstacles])

    import json

    with open("obstacles.json", "w") as write_file:
        json.dump(obsl, write_file, indent=4)
    """

    import json
    with open("obstacles.json") as f:
        data = json.load(f)


    import matplotlib.pyplot as plt

    for i in range(len(data)):
        env = Environment(data[i])
        fig, ax = plt.subplots(1)
        ax.set_xlim(0, env.lx)
        ax.set_ylim(0, env.ly)
        ax.set_aspect('equal')
        for ob in env.obstacles:
            ax.add_patch(plt.Circle((ob.x, ob.y), ob.r, facecolor="gray", edgecolor="k"))
    plt.show()
