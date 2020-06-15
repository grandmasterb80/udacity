# SFND_Unscented_Kalman_Filter - Submission by Daniel Baudisch

## Start

I followed the guide, provided by Udacity Project Description to check out code and to compile it.

## Code Completion

Initially, I completed the code, by adding the code parts from previous exercises:
  - /17. Augmentation Asssignment
  - /20. Sigma Point Prediction Assignment
  - /23. Predicted Mean and Covariance
  - /26. Predict Radar Measurement
  - /29. UKF Update Assignment

In addition, I had to complete
  1. the handling of the very first measurement,
  1. write a predict and update step for the Lidar measurement (corresponding to lesson 4./26. and 4./29. for the Radar measurement),
  1. finalize and complete initialization in constructor (including the adaption of already defined members ==> see "widely off..." comment)

### Handle very first measurement

The handling of the very first measurement is quite straight-forward and described in one of the lessons: we have to map the measurement to a state. For Lidar, we simply assign the x and y coordinates into x - other members of x are set to 0. For Radar, we have to use the state mapping function that was already derived in another lesson - in particular, we convert polar coordinates into cartesian coordinates.

### Predict and Update Steps for Lidar

I wrote the functions by following the predict and update functions for Radar. It is easier to map a Lidar measurement to the state. Therefore, it was pretty simple to create this part.

### Initialization in Constructor

I kept most functionality that I reused from the exercises in functions. Hence, I had to declare some of the intermediate variables as members of the UKF class.

## Testing, Observation and Optimization

After completion of the code, first run was quite promising. At some points of time, the RMSE exceeded the threshold. My observation by looking at the visualization of the highway scenario (green point over vehicles): it was quite obvious that the Uncertainty was too low: when the rear vehicle executes a lane change, the green point was staying in the ego lane for quite a long time.
After adding additional code to compute NIS and maximum RMSE, I modified the P matrix before the predict-and-update is executed. Goal is to add some uncertainty to the current state to ensure that the UKF is more reactive to new measurements. My method to find the right values was, to observe how the states behave compared to ground-truth. First, I added uncertainty to the orientation. To make this more effective, I also had to add uncertainty to the position.
