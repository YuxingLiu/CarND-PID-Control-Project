# PID Control Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The goal of this project is to implement a PID controller in C++ to drive a vehicle around the track in a [simulator](https://github.com/udacity/self-driving-car-sim). Escentially, the PID controller manipulates the steering wheel to minimize the cross track error (CTE).

![](./images/clip1.gif)

---

[//]: # (Image References)

[image1]: ./images/MSE_vs_KpKd.png "Mean Square Error vs. PD Gains"

## PID Parameters Tuning

In general, the proportional control action is the basic component of a PID control to reduce the tracking error, but not quite effective to deal with undesired transient performance (slow response, overshoot, oscillation) and steady-state error. The integral control action is mainly used to reduce the steady-state error, especially in presence of systematic bias or disturbances. The derivative control action provides an early corrective action by anticipating the tracking error, resulting in improved transient performance and stability.

In this project, since the reference trajectory is time varying and small offsets from the lane center is acceptable, the transient performance should be the most important design criterion. Therefore, a PD control is first designed, then the integral control is added to see if it can further improve the control system performance. A combination of manual tuning and discrete optimization is used to determine the PID gains.

### Step 1: Feasible design parameter space

First, several sets of PID gains `(Kp, Ki, Kd)` are tested, in order to identify the feasible range of each parameter. Roughly speaking, a set of PID gains is feasible if the car won't leave the drivable portion of the track. The design space is found to be:

```
0.10 <= Kp <= 0.25
20 <= Kd/Kp <= 50
0 <= Ki <= 0.001
```

### Step 2: PD tuning

A combination of `Kp` and `Kd` is chosen from the design space, which yields the minimum mean square error (MSE) of a complete drive cycle. To exclude the disturbing effect of varying vehicle speeds on the MSE, a PID controller of throttle is applied to maintain a constant vehicle speed at 40 mph. The PID gains are manually tuend as (0.1, 0.0001, 2.0).

To avoid overly large computations, the design space is discretized into a sparse grid. The sets of PD gains and corresponding MSE values are shown in the following figure and table. It can be seen that a large gain `Kp` can effectively reduce MSE, while the open loop zero `-Kd/Kp` should be placed around -30.

![alt text][image1]

| Kp    | Ki    | Kd    | MSE   |
|:-----:|:-----:|:-----:|:-----:| 
| 0.10  | 0     | 2.0   | 1.004 | 
|       |       | 3.0   | 0.851 | 
|       |       | 4.0   | 0.859 | 
|       |       | 5.0   | 0.910 | 
| 0.15  | 0     | 3.0   | 0.536 | 
|       |       | 4.5   | 0.390 | 
|       |       | 6.0   | 0.375 | 
|       |       | 7.5   | 0.444 | 
| 0.20  | 0     | 4.0   | 0.301 | 
|       |       | 6.0   | 0.305 | 
|       |       | 8.0   | 0.325 | 
|       |       | 10.0  | 0.374 | 
| 0.25  | 0     | 5.0   | 0.207 | 
|       |       | 7.5   | 0.188 | 
|       |       | 10.0  | 0.224 | 
|       |       | 12.5  | 0.248 | 
| 0.25  | 1e-6  | 7.5   | 0.211 | 
|       | 1e-5  |       | 0.267 | 
|       | 1e-4  |       | 0.168 | 
|       | 1e-3  |       | 0.143 | 

![](./images/clip2.gif)
