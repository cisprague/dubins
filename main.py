from dubins import evaluate
from solution import solution
try: import matplotlib.pyplot as plt; mpl=True
except: mpl = False
import os, json
from argparse import ArgumentParser
dir = os.path.dirname(os.path.abspath(__file__))

def main(solution_function, plot=False, verbose=False):

    '''E: circular obstacles'''

    # file name
    obsl = dir + "/circ_obs.json"

    # load list of circular obstacle configurations
    with open(obsl) as f:
        obsl = json.load(f)

    # E results
    E = list()

    # test solution function on each configuration
    for obs in obsl:
        E.append(evaluate(solution, obs, verbose=verbose))

    '''C: line obstacles'''

    # file name
    obsl = dir + "/line_obs.json"

    # load list of circular obstacle configurations
    with open(obsl) as f:
        obsl = json.load(f)

    # C results
    C = list()

    # test solution function on each configuration
    for obs in obsl:
        C.append(evaluate(solution, obs, verbose=verbose))

    '''E results'''

    # number of configurations
    ne = len(E)

    # number of successes
    ns = sum([res[7] for res in E])

    # message
    print("Grade E: {}/{} cases passed.".format(ns,ne))

    '''C results'''

    # number of configurations
    nc = len(C)

    # number of successes
    ns = sum([res[7] for res in C])

    # message
    print("Grade C: {}/{} cases passed.".format(ns,nc))

    # plotting
    if plot and mpl:

        # create two column figure
        nr = max(ne, nc)
        fig, ax = plt.subplots(nrows=nr, ncols=2, sharex=True, sharey=True)

        # loop through E results
        for i in range(ne):

            # extract results
            car, xl, yl, thetal, ul, tl, safe, done = E[i]

            # plot obstacles
            for ob in car.obs:
                ax[i, 0].add_patch(plt.Circle((ob[0], ob[1]), ob[2], facecolor="gray", edgecolor="k"))

            # plot origin and target
            ax[i,0].plot(car.x0, car.y0, 'kx')
            ax[i,0].plot(car.xt, car.yt, 'kx')

            # plot trajectory
            ax[i,0].plot(xl, yl, 'k-')

            # zoom to environment area
            ax[i,0].set_xlim(car.xlb, car.xub)
            ax[i,0].set_ylim(car.ylb, car.yub)

            # equal aspect ratio
            ax[i,0].set_aspect("equal")

        # loop through C results
        for i in range(nc):

            # extract results
            car, xl, yl, thetal, ul, tl, safe, done = C[i]

            # plot obstacles
            for ob in car.obs:
                ax[i, 1].add_patch(plt.Circle((ob[0], ob[1]), ob[2], facecolor="gray", edgecolor="k"))

            # plot origin and target
            ax[i,1].plot(car.x0, car.y0, 'kx')
            ax[i,1].plot(car.xt, car.yt, 'kx')

            # plot trajectory
            ax[i,1].plot(xl, yl, 'k-')

            # zoom to environment area
            ax[i,1].set_xlim(car.xlb, car.xub)
            ax[i,1].set_ylim(car.ylb, car.yub)

            # equal aspect ratio
            ax[i,1].set_aspect("equal")

        # set labels
        [ax[-1,i].set_xlabel(r"$x$ [m]") for i in range(2)]
        [ax[i,0].set_ylabel(r"$y$ [m]") for i in range(nr)]

        # set titles
        ax[0,0].set_title("E")
        ax[0,1].set_title("C")

        #fig.savefig("plot.svg", dpi=1000, bbox_inches="tight")

        # show plot
        plt.show()


if __name__ == "__main__":

    # argument parser
    p = ArgumentParser()
    p.add_argument('-p', action='store_true')
    p.add_argument('-v', action='store_true')
    args = p.parse_args()

    # evaluate solution
    main(solution, plot=args.p, verbose=args.v)
