#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {student full name}
# {student id}
# {student email}


def solution(car):

    ''' <<< write your code below >>> '''


    ''' <<< write your code below >>> '''

    return controls, times

if __name__ == "__main__":

    from dubins import evaluate

    # evaluate solution without obstacles in precomputed environment
    car, xl, yl, thetal, ul, tl, done = evaluate(solution, "E", obs=True)
    # evaluate solution with obstacles in precomputed environment
    car, xl, yl, thetal, ul, tl, done = evaluate(solution, "C", obs=True)
    # evaluate solution with obstacles in random environment
    car, xl, yl, thetal, ul, tl, done = evaluate(solution, "C")
