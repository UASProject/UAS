
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
'''
def arm_and_takeoff(aTargetAltitude):
    """Arms vehicle and fly to aTargetAltitude."""
    
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
    
	print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def goto(xdistance, ydistance, gotoFunction=vehicle.simple_goto):
    """
        Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.
        
        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
        the target position. This allows it to be called with different position-setting commands.
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().
        
        The method reports the distance to target every two seconds.
        """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(center)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance
    
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, center)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print("Reached target")
            break;
        time.sleep(2)

'''

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
		












