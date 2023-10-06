# Self-Balancing-Segway-
The code to run a self balancing segway through a pyboard that can also dance. 
A PID controller is used to control the yaw of the robot and keep it balanced whilst a energy detection algorithm detects the music beats.

## Balancing the Segway

### 1. Finding the set-point of the Segway

The purpose of the set-point is to account for the pitch error or pitch offset, usually to counter the centre of mass. For this, the segway was placed in a position where is was able to stand balanced on its wheels, and the pitch angle was recorded. This reading was set as the set point in the code. 

**The pitch offset = set point = 1.23 degrees.**

### 2. PID tuning:

💡 Designing a controller for a given plant or process requires us to determine the correct value for **Kp, Ki and Kd**. This process is known as **“tuning”**

- Tuning Kp: The value of Kp was increased until the system just started oscillating. At this point, it was able to balance with a few seconds with the help of some balancing. Increasing Kp too much resulted in increased response of the controller to changes in error, and increased the overshoot or oscillations.
- Tuning Kd: Kd helped to smooth out the response to the controller in order to reduce overshoot and increase stability. Increasing Kd also increases the sensitivity of the system to changes in error rate. Increasing Kd too much increased the oscillations again, since the controller may have become too sensitive to noise and amplify noise and vibrations in the system.
- Tuning Ki: This helped to reduce the steady state error, which is the drift that remains after the controller has settled. Increasing Ki too much caused the system to act aggressively.

**Final Values:** 

**Kp = 2.46**

**Ki = 0.95**

**Kd = 0.15**

https://drive.google.com/drive/u/0/folders/1YCzb7D612RFHDZSyzSUAEJ5qcaHr-EbN
