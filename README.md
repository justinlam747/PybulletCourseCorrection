# PybulletCourseCorrection
## PyBullet Lane Corridor Follower 
### A Proportional-Integral-Derivative (PID) controller implmented to keep a box/car (entity) aligned with the centerline of a corridor
### This script details a simple PID controller that applies a certain torque rotation based on a output factor from the PID model to acheieve critcally damped movement
### The box is randomly oriented every 5 seconds proving that the PID controller can correct the heading for the box 

## How to Run

To run the simulation, run the main script:

```bash
python main.py
```

## Simulation Results

### Entity before PID adjusts to centerline
![NotAligned](/PIDController/NonAlign.png)

### Entity after PID adjusts to centerline
![Aligned](/PIDController/Align.png)






