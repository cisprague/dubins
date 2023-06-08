# Robotic Planning Assignment 2018

- [Installation](doc/install.md)
- [API](doc/api.md)
- [Assignment](doc/assignment.md)
- [Tutorial](https://cisprague.github.io/kth-dd2410-planning)

## Description

In this assignment we'll consider a constant velocity Dubin's car model, in which the steering angle is the sole control parameter, modelled by the following system of ordinary differential equations:

![](doc/eom.svg)

Our task to **safely** drive the car from its initial position to a target position without colliding with any obstacles or venturing out of bounds. Consider the diagramme below.

![](doc/img/env.svg)
