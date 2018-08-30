#!/usr/bin/env python3
from dubins import evaluate
from solution import solution
try: import matplotlib.pyplot as plt; mpl=True
except: mpl = False
import sys

def plot_results(resl):

    # number of results
    n = len(resl)
    # plots
    fig, ax = plt.subplots(n, sharex=True)
    if n == 1: ax = [ax]
    # for each solution
    for i in range(n):
        # extract data
        car, xl, yl, thetal, ul, tl, done, obs, title = resl[i]
        # plot
        ax[i].plot(xl, yl, "k-")
        ax[i].plot(car.x0, car.y0, "kx")
        ax[i].plot(car.xt, car.yt, "kx")
        ax[i].set_xlim(car.xlb, car.xub)
        ax[i].set_ylim(car.ylb, car.yub)
        ax[i].set_aspect("equal")
        ax[i].set_ylabel(r"$y$ [m]")
        ax[i].set_title(title)
        for ob in car.obs:
            ax[i].add_patch(plt.Circle((ob[0], ob[1]), ob[2], facecolor="gray", edgecolor="k"))
    ax[-1].set_xlabel(r"$x$ [m]")
    plt.show()



def main(plot=False):

    # results
    resl = list()
    # make it to target without obstacles in precomputed environment
    res = evaluate(solution, "E", random=False)
    res += (False, "E | Precomputed")
    resl.append(res)
    print("{0} success: {1}".format(res[-1], res[-3]))

    # make it to target with obstacles in precomputed environment
    res = evaluate(solution, "C", random=False)
    res += (True, "C | Precomputed")
    resl.append(res)
    print("{0} success: {1}".format(res[-1], res[-3]))

    # make it to target with obstacle in random environment
    res = evaluate(solution, "C")
    res += (True, "C | Random")
    resl.append(res)
    print("{0} success: {1}".format(res[-1], res[-3]))

    # make it to target with obstacle in random environment and zero
    # final heading angle
    res = evaluate(solution, "A")
    res += (True, "A | Random")
    resl.append(res)
    print("{0} success: {1}".format(res[-1], res[-3]))

    # plot results if desired and possible
    if plot and mpl:
        plot_results(resl)
    elif plot and not mpl:
        raise ImportError("Please install matplotlib if you want to plot.\nExecute: $ sudo pip3 instlall matplotlib")


if __name__ == "__main__":

    # to plot or not to plot
    try: plot = sys.argv[1]
    except: plot = False
    if plot == "plot": plot = True
    elif plot != False: raise ValueError("Argument can only be 'plot'.")

    # execute main programme
    main(plot)
