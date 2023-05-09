'''  
-------------------------------------------------------
Name: Dancing Segway - Beat Detection
Creator:  Aishwarya Balasundar and Ria Dhopatkar
Date:   20 March 2022
-------------------------------------------------------
This program do the following:
1. Read moves from text file and append into a list
2. Use interrupt to collect samples from mic at 8kHz rate.
3. Fetch  instantenous energy E for 20msec window
4. Obtain sum of previous 50 instanteneous energy measurements
	as sum_energy, equivalent to 1 sec worth of signal.
5. Find the ratio c = instantenous energy/(sum_energy/50)
6. Wait for elapsed time > (beat_period - some margin) 
	since last detected beat
7. Check c value and if higher than BEAT_THRESHOLD
8. When beat is detected, decode the next move in the list of moves and execute
9. REPEAT 2-8

'''
import pyb
from pyb import Pin, Timer, ADC, DAC, LED
import time 
from array import array			# need this for memory allocation to buffers
from oled_938 import OLED_938	# Use OLED display driver
from audio import MICROPHONE
from mpu6050 import MPU6050

#  The following two lines are needed by micropython
#   ... must include if you use interrupt in your program
import micropython
micropython.alloc_emergency_exception_buf(100)

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()				# find the I2C device number
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64,
    external_vcc=False,
    i2c_devid=i2c.scan()[0],
)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Beat Detection')
oled.display()

# define ports for microphone, LEDs and trigger out (X5)

b_LED = LED(4)		# flash for beats on blue LED
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

# Define motion of the motors 
def A_forward(value):
	A1.low()
	A2.high()
	motorA.pulse_width_percent(value)
	
def A_back(value):
	A2.low()
	A1.high()
	motorA.pulse_width_percent(value)
	
def B_forward(value):
	B2.low()
	B1.high()
	motorB.pulse_width_percent(value)

def B_back(value):
	B1.low()
	B2.high()
	motorB.pulse_width_percent(value)
	
def A_stop():
	A1.high()
	A2.high()
	
def B_stop():
	B1.high()
	B2.high()

def flash():		# routine to flash blue LED when beat detected
	b_LED.on()
	pyb.delay(10)
	b_LED.off()
	
# function to decode a move to motor movement from ASCII code in text file
def decode_move(move):

	# tic = pyb.millis()
	# while((pyb.millis() - tic) < 500):
	if move[0] == 'F':
		if move[1] == 'L':
			A_forward(40)
			pyb.delay(100)
			B_stop()

		elif move[1] == 'R':
			B_forward(40)
			pyb.delay(100)
			A_stop()

	elif move[0] == 'B':

		if move[1] == 'L':
			A_back(40)
			pyb.delay(100)
			B_stop()

		elif move[1] == 'R':
			B_back(40)
			pyb.delay(100)
			A_stop()
	    
# function to fetch the next move once a move has been completed upon beat detection	
def next_move(ptr, move_list):
        # Gives moves until out and then gives (0, 0)
		if ptr < len(move_list):
			move = move_list[ptr]
			ptr += 1
			
		else:
			move = (0, 0)
		return move

move_list = [] # array to store all the moves from the text file\


# read moves from text file
f = open ('dance_moves.txt', 'r')
for line in f:
	move_list.append(line)	

ptr = 0 # increases everytime there is a beat to start the next dance move

# Create timer interrupt - one every 1/8000 sec or 125 usec
pyb.disable_irq()
sample_timer = pyb.Timer(7, freq=8000)	# set timer 7 for 8kHz

N = 160				# number of sample to calculate instant energy
mic = ADC(Pin('Y11'))
audio = MICROPHONE(sample_timer, mic, N)
pyb.enable_irq(True)

# Calculate energy over 50 epochs, each 20ms (i.e. 1 sec)
M = 50						# number of instantaneous energy epochs to sum
BEAT_THRESHOLD = 2.7	# threshold for c to indicate a beat
MIN_BEAT_PERIOD = 500	# no beat less than this

# initialise variables for main program loop
e_ptr = 0					# pointer to energy buffer
e_buf = array('L', 0 for i in range(M))	# reserve storage for energy buffer
sum_energy = 0				# total energy in last 50 epochs
oled.draw_text(0,20, 'Ready to GO')	# Useful to show what's happening?
oled.display()
pyb.delay(100)
tic = pyb.millis()			# mark time now in msec

while True:				# Main program loop
	if audio.buffer_is_filled():		# semaphore signal from ISR - set if buffer is full
		
		# Fetch instantaneous energy
		E = audio.inst_energy()			# fetch instantenous energy
		audio.reset_buffer()			# get ready for next epoch

		# compute moving sum of last 50 energy epochs with circular buffer
		sum_energy = sum_energy - e_buf[e_ptr] + E
		e_buf[e_ptr] = E			# over-write earliest energy with most recent
		e_ptr = (e_ptr + 1) % M		# increment e_ptr with wraparound - 0 to M-1
		average_energy = sum_energy/M

		# Compute ratio of instantaneous energy/average energy
		c = E /average_energy
		print(c)
		# oled.draw_text(0,10,'hi={:5.2f}'.format(c))



		if (pyb.millis()-tic > MIN_BEAT_PERIOD):	# if longer than minimum period
			# move = next_move(ptr, move_list)
			if (c>BEAT_THRESHOLD):		# look for a beat
				flash()				# beat found, flash blue LED
				move = move_list[ptr] # execute move in the list
				decode_move(move) # convert ASCII code to motor movement 
				ptr += 1
				
				tic = pyb.millis()	# reset tic
		
		buffer_full = False				# reset status flag
		



