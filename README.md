# PybulletCourseCorrection
## PyBullet Lane Corridor Follower 
### A Proportional-Integral-Derivative (PID) controller implmented to keep a box aligned with the centerline of a corridor
### This script details a simple PID controller that adjusts the torque rotation based on a output factor of the PID to acheieve critcally damped movement
### The box is randomly oriented every 5 seconds proving that the PID controller can correct the heading fo the box no matter what forward angle is appleid

## Car before PID adjusts to centerline
![NotAligned](/PIDController/NonAlign.png)

## Car after PID adjusts to centerline
![Aligned](/PIDController/Align.png)
