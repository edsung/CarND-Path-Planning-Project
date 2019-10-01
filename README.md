# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

Overview
---
This project uses finite state machine and cost functions to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.

### Goals
The goals of this project are the following:
1. The car is able to drive at least 4.32 miles without incident.
2. The car drives according to the speed limit.
3. Max Acceleration and Jerk are not Exceeded.
4. Car does not have collisions.
5. The car stays in its lane, except for the time between changing lanes.
6. The car is able to change lanes
7. There is a reflection on how to generate paths.

### Achieving Goals #1 through #3

Forward motion is achieved by providing waypoint data to plot x and y values for 30, 60, and 90 meters ahead of the ego car. These points are then smoothed out using the spline code recommended by the instructor (line 360 through line 395). This smoothing prevents max acceleration and jerk satisfy the requirements for this project. Goal #2 is achieved by using a ref_vel variable that is initially set at zero, then slowly increased and regularated by the code from line 271 to 295.  

### Achieving Goals # 4 through #6

Finite State Machine was used to achieve the remaining goals. The finite state machine has three state keep lane, change lane left and change lane right. Data from the Sensor_Fusion were looped through (line 132 to 325) to sense cars around ego car.
Code lines 137 to 156 checks to see if there is a car in front of the ego car. If the car is too close ego car, then it will slow down the vehicle and perform lane change. If there are no cars to the left of the ego car then change lane to the left side. The right side of the ego car is checked as well.

### Reflection on how to generate paths.

Path planning starts by setting the initial reference x and y coordinates and yaw (lines 393 to 395). From the previous path's last two point to create the tangential path.(lines 397 to 422). The getXY function is used to generate 30m, 60m, and 90m sets of points to the points list. The points are transformed into ego car's local reference frame (lines 436 to 442). The transformed point are then sent to the spline function to be initialized (line 448). Previous path point are added to the path planning (line 453 to 456). In order to use the path planning to help with limiting the speed, the evenly spaced y-coordinates are needed and achieved by lines 465 to 483. Once path data is then sent to be plotted and path planning is complete.     
