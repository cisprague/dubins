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
In this assignment you're tasked to implement a robotic planning method in order to drive a Dubin's car, with the following dynamics:
```python
dx     = cos(theta)
dy     = sin(theta)
dtheta = tan(phi)
```
from an initial position `(x0,y0)` to a target position `(xt, yt)`, while avoiding both collisions with obstacles and venturing out of bounds.

## Tasks

We'll consider three tasks in order of difficulty:
 - Reach the x-position of the target
 - Reach the position of the target
 - Reach the position of the target with zero heading angle

Using the API, explained below, generate a sequence of steering angle commands `controls` and a sequence of times `times` between which the commands are executed that would yield a collision free and task fulfilling trajectory.

## Solution
In the directory `solution/` you'll find a file named `solution.py`; in this file you should write your code in the sections indicated between `''' <<< write your code below >>> '''` and `''' <<< write your code above >>> '''`. If you feel the need, you may also add ancillary files to `solution/`.

The function `solution()`, within, is expected to return a tuple of the form (`car : Car, controls : list, times : times)`, which will be used to evaluate your implemented planning method's performance upon the execution of `solution.py`, i.e.:
```bash
python3 solution.py
```

## Guidance

In case you're looking for some inspiration or are confused, you may take a look in the directory `examples/` for some ...uhh... examples, which are alterations of the aforementioned `solution.py` file.

If you're having trouble with the technicalities of the code, refer to the API documentation below. When in doubt, read the docs.

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


The second and last method of car, which will be used to grade your performance, is `evaluate(controls, times)`, which takes as its arguments:
 - `controls : list`: sequence of controls `controls[i] : float`
 - `times : list`: sequence of times at which each control is taken `times[i] : float`

and returns a tuple of the form `(xl, yl, thetal, phil, tl, safe, done)` containing:
 - `xl : list`: sequence of x-positions `xl[i] : float`
 - `yl : list`: sequence of y-positions `yl[i] : float`
 - `thetal : list`: sequence of heading angles `thetal[i] : float`
 - `phil : list`: sequence of steering angles `phil[i] : float`
 - `tl : list`: sequence of times `tl[i] : float`
 - `done : tuple`: tuple of success values:
   - `done[0] : bool`: `True` if final x-position is approximately at target x-position, `False` otherwise
   - `done[1] : bool`: `True` if final position is approximately at target position
   - `done[2] : bool`: `True` if final position is approximately at target position and heading angle is close to zero

Note that:
 - Each steering angle `controls[i]` is considered to be constant between `times[i]` and `times[i+1]`, so`controls` must be one element shorter than `times`, i.e. `len(controls) == len(times) - 1`.
 - The initial time must be zeros, i.e. `times[0] == 0`.
 - Each steering angle must be admissible, i.e. `-pi/4 <= controls[i] <= pi/4`.
 - The time sequence must increase, i.e. `times[i+1] > times[i]`.
 - The sequence time-step size must satisfy `times[i+1] - times[i] < 0.01`.
