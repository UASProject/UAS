
import time
import atexit
import RPi.GPIO as GPIO
from pixy import *
from ctypes import *
from dronekit import connect, VehicleMode

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor

mh=Adafruit_MotorHAT()

def turnOffMotors():
	mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(3).run(Adafruit_MotorHAT.RELEASE)
	mh.getMotor(4).run(Adafruit_MotorHAT.RELEASE)

#################  Stepper Control ####################

def lightbulb_detach():
	myStepper=mh.getStepper(200,1)
	myStepper.setSpeed(120)
	for i in range(12):
		myStepper.step(100, Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE)

#################  Servo control ####################


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



#################  Connection Function ####################

def connect_it():

	#connect to PI through serial
	import argparse  
	parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
	parser.add_argument('--connect', 
					   help="vehicle connection target string. If not specified, SITL automatically started and used.")
	args = parser.parse_args()

	connection_string = args.connect

	print("\nConnecting to vehicle on: %s" % connection_string)
	vehicle = connect(connection_string, wait_ready=True)
	return vehicle

#################  Arming Function ####################

def Arm_it(vehicle):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
		print(" Waiting for arming...")
		time.sleep(1)
	
	#print("connected and armed")
		
#################  Takeoff Function ####################		
		
def takeoff(aTargetAltitude):
    """Arms vehicle and fly to aTargetAltitude."""    
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
        #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)


#################  Pixy Control Function ####################

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
			#print 'frame %3d:' % (frame)
			frame = frame + 1
			for index in range (0, count):
				x=blocks[index].x
				y=blocks[index].y
				#print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].type, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height)
				#print(x)
				#print(y)
				time.sleep(1)
				return x,y;

#################  Navigation Function ####################

def Centering():
	while True:
		Xaxis=3;
		Yaxis=3;
		x=5;
		y=6;
		x,y=GetBlocks();
		
		if x< 145:		#sets Xaxis based on Pixy coordinate
			Xaxis= 1;
			print('move west')
		elif x>175:
			Xaxis=-1;
			print('move east')
		else:
			Xaxis=0;
			print('X is centered')

		if y< 105:		#sets Yaxis based on Pixy coordinate
			Yaxis=-1;
			print('move north')
		elif y>135:
			Yaxis=1;
			print('move south')
		else:
			Yaxis=0;
			print('Y is centered')
			
		print('*************************')
			
		if Xaxis==0 and Yaxis==0:		#Exits if within hit box
			print('all centered...SUCK IT KEVIN')
			break;
			'''
		if Xaxis==1:					#gives directions based on axis values
			print('move east')
		elif Xaxis==-1:
			print('move west')
		
		if Yaxis==1:
			print('move north')
		elif Yaxis==-1:
			print('move south')
			'''
	time.sleep(1)
		












