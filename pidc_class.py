'''
-------------------------------------------------------
Name: PID Controller 1
Creator: Peter Cheung
Date: 16 March 2020
Revision: 1.0
-------------------------------------------------------
PID Controller version 1 
This version uses pitch_dot directly from Gyro.
This avoids noisy derivative term and specific to
self-balancing.
Useful for self-balancing challenge
-------------------------------------------------------

This is a modified code of the one given to us by Peter Cheung
'''
import pyb

class PIDC:
    def __init__(self, Kp, Kd, Ki):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.error_last = 0       # These are global variables to remember various states of controller
        self.tic = pyb.millis()
        self.error_sum = 0

    def getPWM(self, target, pitch, pitch_dot):

        # error input
        error = target - pitch          # e[n]
    
        # derivative input
        derivative = -pitch_dot         # negative feedback

        toc = pyb.millis()
        dt = (toc-self.tic)*0.001       # find dt as close to when used as possible
        # Integration input 
        self.error_sum += error*dt            
 
        #   Output 
        PID_output = (self.Kp * error) + (self.Ki * self.error_sum) + (self.Ki * derivative)
        print(PID_output)
        # Store previous values 
        self.error_last = error
        self.tic = toc

        ''' It was found that the segway struggled to balance itself when the pitch angle was very low. This was because the output speed was nearly
        zero when the pitch angle was low, resulting in little response from the motors to balance it. Thus, the minimum speed was capped to 10% PWM
        to ensure this does not happen. '''

        pwm_out1 = min(abs(PID_output), 100)     # make sure the output speed is less than 100
        pwm_out = max(pwm_out1, 10)       # minimum speed capped at 10 
        if PID_output > 0:                              # Output direction (need to check)
            direction = 'forward'
        elif PID_output < 0:
            direction = 'back'
        else: 
            direction = 'stop'

        return pwm_out, direction
