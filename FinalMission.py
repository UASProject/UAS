
from pixy import *
from UAS_Functions import Arm_it, connect_it,Get_Parameters, takeoff,land_it,Get_Home_Loc
from dronekit import connect, VehicleMode, LocationGlobalRelative
import RPi.GPIO as GPIO
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
from MotorFunctions import servo_tighten,servo_loosen,lightbulb_detach,lightbulb_attach,turnOffMotors,TurnOnMotor
import time
import atexit
from ctypes import *
import sys, argparse
from pymavlink import mavutil



mh=Adafruit_MotorHAT()


################################ Get Blocks Function ################################

def GetBlocks(DesiredSig):
    
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
        count = pixy_get_blocks(1, blocks)
	if count > 0:
            # Blocks found #
            #            print 'frame %3d:' % (frame)
            frame = frame + 1
            for index in range (0, count):
                if int(blocks[index].signature)==DesiredSig:
			print("signature = ",DesiredSig)
                    	x=int(blocks[index].x)
                    	y=int(blocks[index].y)

                    	time.sleep(1)   ## **change when perfected
                    	return x,y;
		elif not vehicle.armed==True and count <=0:
       			x=160
			y=100
			print("lost signature")
			return x,y;
			break;
		#index=index +1


################################ Precision Landing  ########################

def send_land_message(vehicle,x,y):
	hor_res=320
	ver_res=200
	hor_fov=75
	ver_fov=47
	msg = vehicle.message_factory.landing_target_encode(
		0, #time_boot_ms (not used)
		0, #garget num
		0, #frame
		(x-hor_res/2)*hor_fov/hor_res,
		(y-ver_res/2)*ver_fov/ver_res,
		0, #altitude. Not supported
		0,0) #size of target in radians
	vehicle.send_mavlink(msg)
	vehicle.flush()

######## Precision Landing with Companion Computer ##############

def Precision_Land(vehicle,DesiredSig):
    	print("Precision Land Function:")
	#x,y= GetBlocks(DesiredSig)
	vehicle.mode=VehicleMode("LAND")
	while vehicle.armed==True:

		
    		if not vehicle.armed==True:
			print("exiting")
			exit(0)
			break;
		else:
			x,y= GetBlocks(DesiredSig)
			send_land_message(vehicle,x,y)
    	time.sleep(1)


######## Precision Loiter with Companion Computer ##############

def Precision_Loiter(vehicle,DesiredSig):
    	print("Precision Loiter Function:")
	#x,y= GetBlocks(DesiredSig)
	for i in range(6):
		
		x,y= GetBlocks(DesiredSig)
		send_land_message(vehicle,x,y)
    		i=i+1
	time.sleep(1)

################################ Yaw ################################

def condition_yaw(vehicle, direction):

    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        0,    # param 1, yaw in degrees (0 is North)
        5,          # param 2, yaw speed deg/s
        direction,          # param 3, direction -1 ccw, 1 cw
        0, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)



############################### Auto North ##################################


def Auto_Yaw(vehicle):
	
	flag=0;
	while 1:
		direction = vehicle.heading
		if direction < 15 or direction > 345:
			flag =1
			print("\nNorth reached\n")
			return flag
			break
		elif direction < 180:
			direction = -1;
		else: 
			direction = 1;
		condition_yaw(vehicle, direction)


############################### Main ##################################


parser=argparse.ArgumentParser(description='parameters')
parser.add_argument('--desiredAlt',help='set desiredAlt')
parser.add_argument('--desiredSig',help='set desiredSig')


args=parser.parse_args()
desiredAlt=float(args.desiredAlt)
desiredSig=float(args.desiredSig)
connection_string=str("127.0.0.1:14550")

pixy_init()

turnOffMotors()
print("motors off")
time.sleep(3)
TurnOnMotor() #Turn on/lock Stepper Motor
servo_tighten() # secure light bulb in band
lightbulb_detach() #unscrew light bulb
lightbulb_attach() #screw in light bulb



vehicle=connect_it(connection_string)    #Connect to Pixhawk via Pi
Get_Parameters(vehicle)        #Get start up parameters of UAS
Arm_it(vehicle)        #arm the motors
time.sleep(3)
print(vehicle.armed)

		
######################################

target=LocationGlobalRelative(29.7063637,-95.4502891,desiredAlt)
takeoff(vehicle,desiredAlt)
time.sleep(3)

print("\ngoing to...\n")
vehicle.simple_goto(target,groundspeed=1)
time.sleep(15)

print("Auto_Yaw")
flag =Auto_Yaw(vehicle) #auto face UAS north

if flag ==1:
	print ("\nTarget Reached.\n")
    Precision_Loiter(vehicle,desiredSig) #loiter on top of signature

        Precision_Land(vehicle,desiredSig)
else:
	time.sleep(1)

time.sleep(5)

print("landing complete")


########## claw action ############
lightbulb_attach()
servo_loosen()


############## Return #############

print("\nre-arming\n")
vehicle.mode=VehicleMode("GUIDED")
Arm_it(vehicle)        #arm the motors
time.sleep(3)

home_target=LocationGlobalRelative(29.7062397,-95.4503102,desiredAlt)

takeoff(vehicle,desiredAlt)
time.sleep(3)


print("going to...")

vehicle.simple_goto(home_target,groundspeed=1)
time.sleep(15)

flag =Auto_Yaw(vehicle)
if flag ==1:
	print ("\nHome Target Reached.\n")
	Precision_Loiter(vehicle,desiredSig)
	Precision_Land(vehicle,desiredSig)	



turnOffMotors()
#exit(0)

