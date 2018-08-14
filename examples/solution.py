# Christopher Iliffe Sprague
# sprague@kth.se

from dubins import Car

''' <<< write your code below >>> '''

from math import pi

def predict(car, x, y, theta, phi, T):

    # seed time
    t = 0

    # list of states, control, and times
    xl     = []
    yl     = []
    thetal = []
    phil   = []
    tl     = []

    # simulate for prescribed time
    while t < T:
        x, y, theta = car.step(x, y, theta, phi)
        t += car.dt
        xl.append(x)
        yl.append(y)
        thetal.append(theta)
        phil.append(phi)
        tl.append(t)

    return xl, yl, thetal, phil, tl

def obstacle_cost(car, xl, yl):

    # closest distance
    dmin = float("inf")

    # iterate through each state
    for x, y in zip(xl, yl):
        # iterate through each obstacle
        for ob in car.obs:
            # relative position
            xr = ob[0] - x
            yr = ob[1] - y
            # distance to obstacle
            d = (xr**2 + yr**2)**0.5
            # if colliding
            if d <= ob[2]:
                return float("inf")
            # if distance is smaller
            elif d < dmin:
                dmin = d

    # return cost
    return 1.0/dmin

def target_cost(car, x, y, theta):

    # relative position to target
    x = car.xt - x
    y = car.yt - y

    # distance to target
    d = (x**2 + y**2)**0.5

    # return cost
    return d

def boundary_cost(car, xl, yl):

    # closest distance
    dmin = float("inf")

    # iterate through each state
    for x, y in zip(xl, yl):
        # distance to north wall
        dn = car.yub - y
        # distance to east wall
        de = car.xlb - x
        # distance to south wall
        ds = car.ylb - y
        # distance to west wall
        dw = car.xub - x
        # minimum distance
        d = min(abs(dn), abs(de), abs(ds), abs(dw))
        if dn < 0 or de < 0 or ds > 0 or dw > 0:
            return float("inf")
        elif d < dmin:
            dmin = d

    # return cost
    return 1.0/dmin

def control(car, x, y, theta):

    # best cost)
    costmin = float("inf")

    # number of steering angles to test
    nphi = 20
    # steering angle increment
    dphi = pi*0.5/nphi
    # iterate through all steering angles
    for phi in [-pi*0.25 + dphi*(i) for i in range(nphi + 1)]:

        # predict trajectory
        xl, yl, thetal, phil, tl = predict(car, x, y, theta, phi, 0.2)

        # compute cost
        cost1 = obstacle_cost(car, xl, yl)
        cost2 = target_cost(car, xl[-1], yl[-1], thetal[-1])
        cost3 = boundary_cost(car, xl, yl)
        cost = cost1 + cost2# + cost3

        # record minimum cost
        if cost <= costmin:
            costmin   = cost
            xlmin     = xl
            ylmin     = yl
            thetalmin = thetal
            philmin   = phil
            tlmin     = tl

    return xlmin, ylmin, thetalmin, philmin, tlmin

def run(car):

    # lists
    xl     = [car.x0]
    yl     = [car.y0]
    thetal = [0]
    phil   = []
    tl     = [0]

    # terminating conditions


    for i in range(1000):

        # best trajectory
        xll, yll, thetall, phill, tll = control(car, xl[-1], yl[-1], thetal[-1])

        # record it
        xl.extend(xll)
        yl.extend(yll)
        thetal.extend(thetall)
        phil.extend(phill)
        tl.extend([t + tl[-1] for t in tll])

    # return lists
    return xl, yl, thetal, phil, tl


''' <<< write your code below >>> '''

def solution():
    car = Car()

    ''' <<< write your code below >>> '''

    xl, yl, thetal, phil, tl = run(car)
    controls = phil
    times = tl

    ''' <<< write your code below >>> '''
    return car, controls, times

if __name__ == "__main__":

    # evaluate your code
    car, controls, times = solution()
    xl, yl, thetal, ul, tl, done = car.evaluate(controls, times)

    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1)
    ax.plot(xl, yl, "k-")
    ax.plot(car.x0, car.y0, 'kx')
    ax.plot(car.xt, car.yt, 'kx')
    for ob in car.obs:
        ax.add_patch(plt.Circle((ob[0], ob[1]), ob[2], facecolor="gray", edgecolor="k"))

    ax.set_aspect("equal")
    ax.set_xlim(car.xlb, car.xub)
    ax.set_ylim(car.ylb, car.yub)
    plt.show()
