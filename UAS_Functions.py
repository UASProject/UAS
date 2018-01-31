import time
import atexit
import RPi.GPIO as GPIO
from pixy import *
from ctypes import *

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor

mh=Adafruit_MotorHAT()

def turnOffMotors():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

def lightbulb_detach():
	myStepper=mh.getStepper(200,1)
	myStepper.setSpeed(120)
	for i in range(12):
		myStepper.step(100, Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE)

def servo_close():
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(7,GPIO.OUT)

	for i in range(20):
		GPIO.output(7,1)
		time.sleep(0.0005)
		GPIO.output(7,0)
		time.sleep(.1)

	for n in range(20):
		GPIO.output(7,1)
		time.sleep(0.0015)
		GPIO.output(7,0)
		time.sleep(.1)

	for f in range(20):
		GPIO.output(7,1)
		time.sleep(0.0025)
		GPIO.output(7,0)
		time.sleep(.1)

	GPIO.cleanup()


def GetBlocks():

	class Blocks (Structure):
		fields_ = [ ("type", c_uint),
		("signature", c_uint),
		("x", c_uint),
		("y", c_uint),
		("width", c_uint),
		("height", c_uint),
		("angle", c_uint) ]
	
	blocks = BlockArray(100)
	frame  = 0
	
	# Wait for blocks #
	while 1:
	
		count = pixy_get_blocks(100, blocks)
		
		if count > 0:
		# Blocks found #
			print 'frame %3d:' % (frame)
			frame = frame + 1
			for index in range (0, count):
				x=blocks[index].x
				y=blocks[index].y
				print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].type, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height)
				print(x)
				print(y)
				time.sleep(.5)
				return x,y;



