# API

In this assignment we'll work with a `mission` object, which encapsulates the dynamics and environment of this problem.

## Attributes
Attribute           | Type                     | Description |
| ----------------- | ------------------------ | ---------------- |
|`mission.origin`   | `np.ndarray((1,2))`      | Initial position |
|`mission.target`   | `np.ndarray((1,2))`      | Target position |
|`mission.xbound`   |  `np.ndarray((1,2))` |  Horizontal boundaries, where`xbound[0]` is the lower bound and `xbound[1]` the upper. |
|`mission.ybound`   |  `np.ndarray((1,2))` |  Vertical boundaries, where`ybound[0]` is the lower bound and `ybound[1]` the upper. |   |   |   |
|`mission.state`    | `np.ndarray(shape(1,3))` | Current state, where <li> `state[0]`: horizontal position <li> `state[1]`: vertical position <li> `state[2]`: heading angle|
|`mission.time`     | `float`                  | Current time |
|`mission.states`   | `np.ndarray((n,3))`      | Record of `n` states, where: <li> `states[:,0]`: horizontal positions <li> `states[:,1]`: vertical positions <li>`states[:,2]`: heading angles |
|`mission.times`    | `np.ndarray((1,n))`      | Record of `n` times |
|`mission.controls` | `np.ndarray((1,n-1))`      | Record of `n-1` controls |

## Methods

| Method                      | Arguments | Returns | Notes |
| --------------------------- | --------- | ------- | ----------- |
| `mission.set(s, t)`         | <li> `s : np.ndarray((1,3))`: state <li> `t : float`: time | None | Sets <li> `mission.state` to `s` <li> `mission.time` to `t` |
| `mission.reset()`           | None  | None | Resets <li>`mission.state` <li> `mission.time` <li> `mission.states` <li> `mission.times` <li> `mission.controls` |
| `mission.record(s, t, u)`   |  <li> `s : np.ndarray((n,3))`: state <li> `t : float or np.ndarray((1,n))`: time <li> `u : float or np.ndarray((1,n))`: control  | None |  Appends <li> `s` to `mission.states` <li> `t` to `mission.times` <li> `u` to `mission.controls` |
| `mission.safe(p0, p1=None)` |   <li> `p0 : np.ndarray((1,2))`: first position <li> `p1 : np.ndarray((1,2))`: second position |  <li> If only given `p0`: `True` if `p0` is within boundaries and in obstacle free space, `False` otherwise <li> If given both `p0` and `p1`: `True` if line does not intersect neither boundaries nor obstacles, `False` otherwise.|  None |
| `mission.done(p)`           |  <li> `p : np.ndarray((1,2))`: position  | `True` if `p` is approximately at `mission.target` |  None |
| `mission.step(u, Dt=None, inplace=False, verbose=False, record=False)`   | <li> `u : float or callable`: control or control function of the form `control(t, s)` <li> `Dt : float`: duration to simulate <li> `inplace : bool`: set the internal state and time <li> `verbose : bool`: if `True` print simulation information <li> `record : bool`: append `s`, `u`, and `t` to the records | If `u` is a float, tuple containing: <li> `s : np.ndarray((1,3))`: new state <li> `u : float`: control <li> `t : float`: new time <li> `safe : bool`: `True` if transition was safe, `False` otherwise <li> `done : bool`: `True` if new position is approximately at target, `False` otherwise <br><br> If `Dt` is given, tuple containing: <li> `s : np.ndarray((n,3))`: states  <li> `u : np.ndarray((1, n))`: controls  <li> `t : np.ndarray((1,n))`: times <li> `safe : bool`: `True` if transition was safe, `False` otherwise <li> `done : bool`: `True` if new position is approximately at target, `False` otherwise | <li> If `inplace` is `True` than the final state and time are set. <li> If `verbose` is `True`, the state and time are shown.|
| `mission.simulate()`   | <li> `u : np.ndarray((1,n-1))`: sequence of controls <li> `t : np.ndarray((1,n))`: sequence of times  | Returns the terminal percent distance to the target, i.e. `1-d/D`, where `D` is the distance between the origin and target and `d` is the distance from the car to the target. |  The returned quantity signifies your success. |
|`mission.plot_traj(ax=None)`   |  `ax : matplotlib.axes`: optional preexisting axis plot on.| If `ax` is not given, returns a tuple `(fig : matplotlib.figure, ax : matplotlib.axes)`, nothing otherwise  | Plots the environment and trajectory from the `states` record |
|`mission.plot_records(ax=None)`| `ax : matplotlib.axes`: optional preexisting axis plot on.  | If `ax` is not given, returns a tuple `(fig : matplotlib.figure, ax : matplotlib.axes)`, nothing otherwise  | Plots timeline of the records from `states`, `times` and `controls`. |

## Implementation
The `mission` API, as described above, allows one to flexibly test planning methods. Let's walk through how to use it.

First of all, we need to import the module for this assignment, after which we can instantiate the `mission` object.
```python
# import necessary module
from dd2410planning import mission
# instantiate mission object
mis = mission()
```

We can plot the environment to get more of a sense of it
```python
>>> fig, ax = mis.plot_traj()
>>> fig.show()
```
![](env.svg)

For demonstration, let's consider a constant control and a control function
```python
import numpy as np
# constant control
u = np.random.uniform(-0.1, 0.1)
# control function
uf = lambda t, s: np.random.uniform(-0.1, 0.1)
```

With either of these we can simulate one step in time, with the time-step size determined by an adaptive numerical integrator:
```python
>>> mis.step(u)
(array([ 0.1704, 23.6773, -0.0011]),
 -0.017698276623792975,
 0.07042401979601698,
 True,
 False)

>>> mis.step(uf)
(array([ 0.1704, 23.6772, -0.0025]),
 -0.03742425078813959,
 0.07042401979601698,
 True,
 False)

```

We can also step for a specific duration, e.g. one second
```python
>>> mis.step(u, Dt=1)
(array([[ 0.1   , 23.6773,  0.    ],
        [ 0.1764, 23.6772, -0.0015],
        [ 0.2528, 23.6771, -0.0028],
        [ 0.3528, 23.6767, -0.0046],
        [ 0.4528, 23.6762, -0.0064],
        [ 0.5528, 23.6754, -0.0082],
        [ 0.6528, 23.6745, -0.0099],
        [ 0.7528, 23.6734, -0.0117],
        [ 0.8528, 23.6722, -0.0135],
        [ 0.9528, 23.6707, -0.0152],
        [ 1.0528, 23.6691, -0.017 ],
        [ 1.0999, 23.6683, -0.0178]]),
 array([-0.0177, -0.0177, -0.0177, -0.0177, -0.0177, -0.0177, -0.0177,
        -0.0177, -0.0177, -0.0177, -0.0177]),
 array([0.    , 0.0764, 0.1528, 0.2528, 0.3528, 0.4528, 0.5528, 0.6528,
        0.7528, 0.8528, 0.9528, 1.    ]),
 True,
 False)
```
With `mission.step` we can use the `inplace` argument to internally set the final conditions as well
```python
>>> mis.state, mis.time
(array([ 0.1   , 12.5701,  0.    ]), 0.0)
>>> state, control, time, safe, done = mis.step(u, Dt=1)
>>> mis.state, mis.time
(array([ 0.1   , 12.5701,  0.    ]), 0.0)
>>> state, control, time, safe, done = mis.step(u, Dt=1, inplace=True)
>>> mis.state, mis.time
(array([ 1.1   , 12.5771,  0.0179]), 1.0)
>>> mis.reset()
>>> mis.state, mis.time
(array([ 0.1   , 12.5701,  0.    ]), 0.0)
```

One can keep track of the car's trajectory either trough the `step` or `record` as such
```python
>>> fig0, ax0 = mis.plot_traj()
>>> fig1, ax1 = mis.plot_records()
>>> for i in range(20):
        mis.step(uf, Dt=50, record=True)
        mis.plot_traj(ax0)
        mis.plot_records(ax1)
        mis.reset()
>>> fig.show()
```
![](img/random_traj.svg)
![](img/random_records.svg)

It should be noted here that `step` stops once either `safe` is `False` or `done` is `True` regardless of `Dt`, i.e. when an obstacle is intersected or when the target position is achieved.
