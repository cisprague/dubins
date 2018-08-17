# Dubin's Car

## Installation
Ensure that you have working version of [Python 3](https://www.python.org/downloads/) and pip installed. <br>
Execute the following in your terminal to download and install the code:

```bash
git clone https://github.com/cisprague/dubins.git
cd dubins
sudo pip3 install .
```

## Description
In this assignment you're tasked to implement a robotic planning method in order to drive a Dubin's car, with the following dynamics,
```python
dx     = cos(theta)
dy     = sin(theta)
dtheta = tan(phi)
```
, from an initial position `(x0,y0)` to a target position `(xt, yt)`, while avoiding both collisions with obstacles and venturing out of bounds.

## Tasks

We'll consider three graded tasks in order of difficulty:
 - **E**: Reach the target without obstacles.
 - **C**: Reach the target with obstacles.
 - **A**: Reach the target with obstacles, and with a final heading angle of zero (optional).

Using the API, explained below, generate a sequence of steering angle commands `controls` and a sequence of times `times`, between which the commands are executed, that would yield a collision free and task fulfilling trajectory.

## Solution
Create a file named `solution.py`,
```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# {student full name}
# {student id}
# {student email}
def solution(car):
  '''
  Din kod här
  '''
  return controls, times
```
, containing a function `solution` that receives a `Car` object `car` and returns a tuple containing,
 - `controls : list`: sequence of steering angles `controls[i] : float`
 - `times : list`: sequence of times at which the controls are executed `times[i] : float`

, where it should be noted that `controls[i]` is considered to be constant between `times[i]` and `times[i+1]`, hence `len(controls) == len(times) - 1`. If needed, you may add ancillary code outside `solution(car)` within the `solution.py` file.

Your `solution` function should implement a robotic planning method, with the use of the attributes and methods of `Car`, to produce a sequence of steering angles and times at which they're executed, that would drive the car successfully in accordance to the previously detailed tasks.

Note that obstacles, origin position, and target position are randomised upon initialisation of `Car`, so each `car` is different.

You can evaluate your performance using the `evaluate` method of `Dubins` in the following way:

```python
from dubins import evaluate
# evaluate solution function at E level with precomputed obstacles (same as Kattis)
xl, yl, thetal, ul, tl, done = evaluate(solution, "E", random=False)
print("Success?", done)
# evaluate solution function at C level with random obstacles
xl, yl, thetal, ul, tl, done = evaluate(solution, "C", random=True)
print("Success?", done)
```

## API

### Car

In this assignment, we'll work exclusively with the `Car` object, which you can import and instantiate as follows:

```python
from dubins import Car
car = Car()
```

The `Car` object has several attributes which you may find useful, namely:
 - `x0 : float`: initial x-position [m]
 - `y0 : float`: initial y-position [m]
 - `xt : float`: target x-position [m]
 - `yt : float`: target y-position [m]
 - `dt : float`: time-step size [s]
 - `xlb : float`: minimum x-position [m]
 - `xub : float`: maximum x-position [m]
 - `ylb : float`: minimum y-position [m]
 - `yub : float`: maximum y-position [m]
 - `obs : list`: list of tuples for each obstacle `obs[i]`, where:
   - `obs[i][0] : float`: x-position [m]
   - `obs[i][1] : float`: y-position [m]
   - `obs[i][2] : float`: radius [m]

The method of `Car` that you'll need to utilise in your implementation of robotic planning methods is `step(x, y, theta, phi)`, which takes as its arguments:
 - `x : float`: x-position
 - `y : float`: y-position
 - `theta : float`: heading angle
 - `phi : float`: steering angle

and returns a tuple of the form `(xn, yn, thetan)`, containing:
 - `xn : float`: new x-position
 - `yn : float`: new y-position
 - `thetan : float`: new heading angle

### Evaluate

The function which will be used to grade your performance, is `evaluate(solution_function, grade, obs)`, which takes as its arguments:
 - `solution_function : callable`: your solution function
 - `grade : str`: grade level at which to grade your function
 - `random : bool`: use random obstacles if `True`, otherwise use precomputed obstacles from Kattis

and returns a tuple of the form `(xl, yl, thetal, phil, tl, safe, done)` containing:
 - `car : Car`: car object for which your solution function was evaluated
 - `xl : list`: sequence of x-positions `xl[i] : float`
 - `yl : list`: sequence of y-positions `yl[i] : float`
 - `thetal : list`: sequence of heading angles `thetal[i] : float`
 - `phil : list`: sequence of steering angles `phil[i] : float`
 - `tl : list`: sequence of times `tl[i] : float`
 - `done : bool`: success of trajectory for corresponding grade:
   - `grade == "E"`: `True` if final x-position is approximately at target x-position
   - `grade == "C"`: `True` if final position is approximately at target position
   - `grade == "A"`: `True` if final position is approximately at target position and heading angle is close to zero

Note that:
 - Each steering angle `controls[i]` is considered to be constant between `times[i]` and `times[i+1]`, so`controls` must be one element shorter than `times`, i.e. `len(controls) == len(times) - 1`.
 - The initial time must be zeros, i.e. `times[0] == 0`.
 - Each steering angle must be admissible, i.e. `-pi/4 <= controls[i] <= pi/4`.
 - The time sequence must increase, i.e. `times[i+1] > times[i]`.
 - The sequence time-step size must satisfy `times[i+1] - times[i] < 0.01`.
