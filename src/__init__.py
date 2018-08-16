# Christopher Iliffe Sprague
# sprague@kth.se

from math import cos, sin, tan, pi, atan2
from random import uniform

def evaluate(solution_function):

    # initialise environment with non-random obstacles
    env = Environment(static=True)

    # initialise car with that environment
    car = Car(env)

    # execute student's solution function
    car, controls, times = solution_function(Car(env=Environment(static=True)))

    # return state, control, and time lists, along with doneness
    return car.evaluate(controls, times)


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
        self.y0 = uniform(self._environment.ly*0.2, self._environment.ly*0.8)

        # random target position [m]
        self.xt = self._environment.lx
        self.yt = uniform(self._environment.ly*0.2, self._environment.ly*0.8)

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

    def __init__(self, static=False):

        # horizontal length
        self.lx  = float(20)
        # vertical length
        self.ly  = float(10)
        # grading
        if static:
            self.static_obstacles()
        # initialise obstacles
        else:
            self.init_obstacles()

    def static_obstacles(self):

        # obstacles (x, y, r)
        xyr = [
            (9.04171271281707,   1.9271911363055318, 0.7137837905980843),
            (14.84907899973128,  4.630260206468853,  0.6423293234264171),
            (6.817542822242217,  4.01075837413714,   0.672704444726865 ),
            (7.343910742259128,  8.744871479451902,  0.5184571088715749),
            (14.494430975920748, 9.145654242269735,  0.6705858892897931),
            (10.236457126137953, 8.675064781200511,  0.6878464659319687),
            (11.537989018666813, 1.576169582576007,  0.6255341639413847),
            (4.819744789035905,  1.4672518733985913, 0.742139163004419 ),
            (4.719779913624635,  7.348174223848854,  0.5676719711424706),
            (8.206381652304799,  6.287894928491917,  0.7888866679675341),
            (14.13918384312909,  2.095848535090256,  0.7004377073903887),
            (4.731084104774632,  5.026939570950342,  0.633763184590268 ),
            (12.814150521971364, 6.957670581317933,  0.5678568860233869),
            (11.793013445977131, 4.355559939468833,  0.5553529571338907),
            (15.368765215214793, 6.932086094545404,  0.546487541545826 ),
            (10.564329825426231, 6.0130274460901045, 0.5050711132562814),
            (4.541424807625523,  9.475893771992098,  0.5105955198301957),
            (9.4165156346914,    4.247951838436274,  0.5646356791454193),
            (7.08713680373352,   0.6266893737309273, 0.5557241167556356)
        ]

        # assign obstacles
        self.obstacles = [Obstacle(*ob) for ob in xyr]

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
                    print(i)
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

    env = Environment(True)
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1)
    ax.set_xlim(0, env.lx)
    ax.set_ylim(0, env.ly)
    ax.set_aspect('equal')
    for ob in env.obstacles:
        ax.add_patch(plt.Circle((ob.x, ob.y), ob.r, facecolor="gray", edgecolor="k"))
    plt.show()

    print([(ob.x, ob.y, ob.r) for ob in env.obstacles])
