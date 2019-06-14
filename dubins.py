# Christopher Iliffe Sprague
# sprague@kth.se

from math import cos, sin, tan, pi
from random import uniform

def evaluate(solution_function, obs, verbose=False):

    # initialise environment
    env = Environment(obs=obs)

    # initialise car with that environment
    car = Car(env)

    # execute student's solution function
    controls, times = solution_function(car)

    # return state, control, and time lists, along with doneness and car
    xl, yl, thetal, ul, tl, safe, done = car.evaluate(controls, times, verbose=verbose)
    return car, xl, yl, thetal, ul, tl, safe, done


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

    def evaluate(self, controls, times, verbose=False):

        '''
        Returns a simulated final result, given a sequence of controls
        and times. controls[i] is considered constant between times[i]
        and times[i+1].
        '''

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

                if verbose:
                    print("x:", xn, "y:", yn, "theta:", thetan)

                # record state
                xl.append(xn)
                yl.append(yn)
                thetal.append(thetan)

                # record control
                ul.append(controls[i])

                # record time
                tl.append(tl[-1] + self.dt)

                # get safety and doneness
                safe = self._environment.safe(xn, yn)
                done = True if ((self.xt-xn)**2 + (self.yt-yn)**2)**0.5 < 1 else False

                # terminate if safe or done
                if not safe:
                    term = True
                    break
                elif done:
                    term = True
                    break
                else:
                    term = False

            if term == True:
                break

        # extend last control to signify constant value between times
        ul.append(ul[-1])

        # return states, controls, times, final saftey, and done
        return xl, yl, thetal, ul, tl, safe, done

class Environment(object):

    '''
    Environment described by a rectangle [(0,0),(Lx,0),(Lx,Ly),(0,Ly)],
    containing n circular obstacles of radius r, positioned randomly
    within.
    '''

    def __init__(self, obs=None):

        # horizontal length - these values work well
        self.lx  = float(20)
        # vertical length
        self.ly  = float(10)

        # initialise obstacles randomly
        if obs is None:
            self.init_obstacles()

        # intentionally no obstacles
        elif obs is False:
            pass

        # initialise supplied obstacles
        else:
            self.obstacles = [Obstacle(*ob) for ob in obs]

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
