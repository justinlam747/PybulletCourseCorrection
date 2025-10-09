# PybulletCourseCorrection
## PyBullet Lane Corridor Follower 
### A Proportional-Integral-Derivative (PID) controller implmented to keep a box/car (entity) aligned with the centerline of a corridor
### This script details a simple PID controller that applies a certain torque rotation based on a output factor from the PID model to acheieve critcally damped movement
### The box is randomly oriented every 5 seconds proving that the PID controller can correct the heading for the box 

## Simulation Results

### Entity before PID adjusts to centerline
![NotAligned](/PIDController/NonAlign.png)

### Entity after PID adjusts to centerline
![Aligned](/PIDController/Align.png)

## How to Run

To run the simulation, simply execute the main script:

```bash
python main.py
```

## How It Works

The PID controller continuously adjusts the entity’s orientation by applying an angular torque, thus reseting orientation back to the corridor’s centerline.  

It computes the "control output" using:

\[
u(t) = K_p e(t) + K_i \int e(t)\,dt + K_d \frac{de(t)}{dt}
\]

Where:

- **Kp** – Proportional gain (response to current error)  
- **Ki** – Integral gain (response to accumulated error)  
- **Kd** – Derivative gain (response to rate of error change)

The resulting torque is applied along the **z-axis** to adjust(steer) the entity back toward the centralline (Yaw adjustment).

### How the Terms Are Estimated

**Proportional Term (`Kp * e(t)`):**  
Calculated directly from the current offset between the entity’s center and the corridor’s midpoint.

**Integral Term (`Ki * ∫e(t)dt`):**  
In the simulation, we **approximate** the integral by **adding up small error values** over time(where dt is each stem in time 1/240).  
For example:
```python
integral += error * dt


