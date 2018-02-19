# PID Control Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The goal of this project is to implement a PID controller in C++ to drive a vehicle around the track in a [simulator](https://github.com/udacity/self-driving-car-sim). Escentially, the PID controller manipulates the steering wheel to minimize the cross track error (CTE).

![](./images/clip1.gif)

---

[//]: # (Image References)

[image1]: ./images/MSE_vs_KpKd.png "Mean Square Error vs. PD Gains"

## PID Parameter Tuning




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

![alt text][image1]
