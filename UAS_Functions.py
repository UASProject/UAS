
import time
import atexit
import RPi.GPIO as GPIO
from pixy import *
from ctypes import *
from dronekit import connect, VehicleMode
from pymavlink import mavutil
'''
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
	myStepper.run(mh.RELEASE)

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

'''

#################  Connection Function ####################

def connect_it(connection_string):

	#connect to PI through serial

	print("\nConnecting to vehicle on: %s" % connection_string)
	vehicle = connect(connection_string, wait_ready=True)
	return vehicle

#################  Arming Function ####################

def Arm_it(vehicle):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        print(vehicle.is_armable)
	time.sleep(1)
    

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
		print(" Waiting for arming...")
		time.sleep(1)
	
	#print("connected and armed")

#################  Parameters Function ####################

def Get_Parameters(vehicle):
    print "Get some vehicle attribute values:"
    print " GPS: %s" % vehicle.gps_0
    print " Battery: %s" % vehicle.battery
    print " Last Heartbeat: %s" % vehicle.last_heartbeat
    print " Is Armable?: %s" % vehicle.is_armable
    print " System status: %s" % vehicle.system_status.state
    print " Mode: %s" % vehicle.mode.name    # settable
		
#################  Takeoff Function ####################		
		
def takeoff(vehicle,aTargetAltitude):
    """Arms vehicle and fly to aTargetAltitude."""    
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.925:
        
        #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

#################  Landing Function ####################

def land_it(vehicle):
	print("landing...")
	vehicle.mode = VehicleMode("LAND")
	#print("Returning to Launch Position")
	#vehicle.mode = VehicleMode("RTL")





'''

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
	
	blocks = BlockArray(3)
	frame  = 0
	
	# Wait for blocks #

	while 1:
		count = pixy_get_blocks(3, blocks)
		if count > 0:
		# Blocks found #
#			print 'frame %3d:' % (frame)
			frame = frame + 1
			for index in range (0, count):
				x=int(blocks[0].x)
				y=int(blocks[0].y)
				g=int(blocks[0].signature)
				x2=int(blocks[1].x)
				y2=int(blocks[1].y)
				g2=int(blocks[1].signature)


#				print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].type, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height)
#				print("x1",x)
#				print("x2",x2)
#				print("y1",y)
#				print("signature: ",g)
#				print("count: ",count)
				t=blocks[index]
				time.sleep(1)
				return x,y,g,x2,y2,g2;
	
#################  Navigation Function ####################

def Centering():
	print("Centering Function:")
	while True:
#		Xaxis=3;
#		Yaxis=3;
#		x=5;
#		y=6;
		
#		North=-.5
#		South=.5
#		West=-.5
#		East=.5
#		Zaxis=0
#		duration=.5
				
		x,y,g,x2,y2,g2=GetBlocks();
		print("x",x,"y",y,"g",g,"x2",x2,"y2",y2,"g2",g2)
#		print(x,y)
						
		if x< 145:		#sets Xaxis based on Pixy coordinate
			Xaxis= 1;
			print('move west')
						#send_ned_velocity(west, 0, Zaxis, duration)
						
		elif x>175:
			Xaxis=-1;
			print('move east')
						#send_ned_velocity(East, 0, Zaxis, duration)
		else:
			Xaxis=0;
			print('X is centered')

		if y< 105:		#sets Yaxis based on Pixy coordinate
			Yaxis=-1;
			print('move north')
								#send_ned_velocity(0, North, Zaxis, duration)
		elif y>135:
			Yaxis=1;
			print('move south')
						#send_ned_velocity(0, South, Zaxis, duration)
		else:
			Yaxis=0;
			print('Y is centered')
						
		print('*************************')
						
		if Xaxis==0 and Yaxis==0:		#Exits if within hit box
			print('all centered...SUCK IT KEVIN')
			break;

		time.sleep(1)
		
'''

def send_ned_velocity(vehicle,velocity_x, velocity_y, velocity_z, duration):
    """
        Move vehicle in direction based on specified velocity vectors.
        """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
           0,       # time_boot_ms (not used)
           0, 0,    # target system, target component
           mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
           0b0000111111000111, # type_mask (only speeds enabled)
           0, 0, 0, # x, y, z positions (not used)
           velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
           0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
           0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink))

# send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)








