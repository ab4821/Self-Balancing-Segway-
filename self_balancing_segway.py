'''  
-------------------------------------------------------
Name: Self Balancing Segway
Creator:  Aishwarya Balasundar and Ria Dhopatkar
Date:   20 March 2022
-------------------------------------------------------
This program do the following:
1. Enables adjustment of target angle, Kp, Kd and Ki through the potentiometer
2. Uses PID control to calculate the speed of the motor required to balance the segway 

'''


import pyb
import time
from pyb import LED, Pin, Timer
from pidc import PIDC
from oled_938 import OLED_938
from mpu6050 import MPU6050

# Define 5k Potentiometer
pot = pyb.ADC(Pin('X11'))

# Define pins to control motor
A1 = Pin('X3', Pin.OUT_PP)		# Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				# Control speed of motor A
B1 = Pin('X7', Pin.OUT_PP)		# Control direction of motor B
B2 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				# Control speed of motor B

# Configure timer 2 to produce 1KHz clock for PWM control
tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()				# find the I2C device number
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64, external_vcc=False, i2c_devid=i2c.scan()[0],
)
oled.poweron() #initialising the oled
oled.init_display()

trigger = pyb.Switch()
scale = 4.0 #setting the maximum value for each of the scales
scale1 = 10.0 
scale2 = 2.0
scale3 = 1.0

while not trigger():
     time.sleep(0.001) 
     target = pot.read() * scale / 4095  #first potentiometer value changes target angle
     oled.draw_text(0,50,'Kd = {:5.2f}'.format(target)) #allows target angle to displayed
     oled.display()
while trigger(): pass
while not trigger():
    time.sleep(0.001)
    k_p = pot.read() * scale1 / 4095 #second potentiometer value changes K_p
    oled.draw_text(0,30,'Kp = {:5.2f}'.format(k_p)) #allows Kp to be displayed
    oled.display()
while trigger(): pass
while not trigger():
    time.sleep(0.001)
    k_i = pot.read() * scale2 / 4095 #third potentiometer value changes K_i
    oled.draw_text(0,40,'Ki = {:5.2f}'.format(k_i)) #allows ki to be displayed
    oled.display()
while trigger(): pass  
while not trigger():
    time.sleep(0.001)
    k_d = pot.read() * scale3 / 4095 #first potentiometer value changes k_d
    oled.draw_text(0,50,'Kd = {:5.2f}'.format(k_d))  #allows kd to be displayed
    oled.display()
while trigger(): pass

#final values
# k_p = 2.4
# k_i = 0.1
# k_d = 0.5

# IMU connected to X9 and X10
imu = MPU6050(1, False)    	# Use I2C port 1 on Pyboard
segway = PIDC(k_p, k_d, k_i)

def pitch_angle(pitch, dt, alpha): #find pitch angle and pitch_dot
	angle = imu.pitch() 
	pitch_dot = imu.get_gy() 
	pitch = alpha*(pitch + pitch_dot*dt) + (1-alpha)*angle #applying complementary filter
	return (pitch, pitch_dot)

alpha = 0.99
pitch = 0
set_point = 1.3 #angle Segway naturally balances at

# try: 
tic1 = pyb.millis()

while True:
    dt = (pyb.millis() - tic1) * 0.001        

    # if (dt > 5000):
    (pitch, pitch_dot) = pitch_angle(pitch, dt, alpha)
    (speed, dir) = segway.getPWM(set_point, pitch, pitch_dot) 
    tic1 = pyb.millis()
    
    motorA.pulse_width_percent(speed) #controlling speed of motors with PWM signal
    motorB.pulse_width_percent(speed)

    if (dir == "forward"):
        A1.high()
        A2.low()
        B1.low()
        B2.high()
    
    elif (dir == "back"):
        A1.low()
        A2.high()
        B1.high()
        B2.low()

    elif (dir == "stop"):
        A1.high(), A2.high()
        B1.high(), B2.high()