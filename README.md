# Dubin's Car

## Installation
After ensuring that you have a working version of Python 3 and Git, you can obtain the source code by cloning this repository as so:
```bash
git clone https://github.com/cisprague/dubins.git
```

## Description
In this assignment you're tasked to implement a robotic planning method in order to drive a Dubin's car, with the following dynamics,
```python
x[t+1]     = x[t]     + cos(theta[t])
y[t+1]     = y[t]     + sin(theta[t])
theta[t+1] = theta[t] + tan(phi[t])
```
, from an initial position `(x0,y0)` to a target position `(xt, yt)`, while avoiding both collisions with obstacles and venturing out of bounds.

The state variables are:
 - `x`: horizontal position
 - `y`: vertical position
 - `theta`: heading angle (direction of travel)

And the sole control variable is the steering angle `phi ∈ [-pi/4, pi/4]`.

## Tasks

We'll consider two graded tasks in order of difficulty:
 - **E**: Reach the target with spherical obstacles.
 - **C**: Reach the target with line obstacles

Using the API, explained below, generate a sequence of steering angle commands `controls` and a sequence of times `times`, between which the commands are executed, that would yield a collision free and task fulfilling trajectory.

You should end up with a solution that looks something like this:

![](plot.svg)

## Solution
Submit a file named `solution.py`,
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

Note that:
 - Each steering angle `controls[i]` is considered to be constant between `times[i]` and `times[i+1]`, so`controls` must be one element shorter than `times`, i.e. `len(controls) == len(times) - 1`.
 - The initial time must be zeros, i.e. `times[0] == 0`.
 - Each steering angle must be admissible, i.e. `-pi/4 <= controls[i] <= pi/4`.
 - The time sequence must increase, i.e. `times[i+1] > times[i]`.
 - The sequence time-step size must be less than 0.01, i.e. `times[i+1] - times[i] > 0.01`.


You can evaluate your solution by executing the following terminal command from within the dubins directory:

```bash
>>> python3 main.py
Grade E: 6/6 cases passed.
Grade C: 6/6 cases passed.
```

You may also supply additional flags as so:
```bash
# show a plot
python3 main.py -p
# print trajectory information
python3 main.py -v
```


## API

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
