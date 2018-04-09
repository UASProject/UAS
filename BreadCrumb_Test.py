
from pixy import *
from UAS_Functions import Arm_it, connect_it,Get_Parameters, takeoff,land_it,send_ned_velocity
from dronekit import connect, VehicleMode #, LocationGlobalRelative
import RPi.GPIO as GPIO
import time
import atexit
from ctypes import *
import sys, argparse
from pymavlink import mavutil






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
        count = pixy_get_blocks(3, blocks)
	if count > 0:
            # Blocks found #
            #            print 'frame %3d:' % (frame)
            frame = frame + 1
            for index in range (0, count):
                if int(blocks[index].signature)==DesiredSig:
		    print(DesiredSig)
                    x=int(blocks[index].x)
                    y=int(blocks[index].y)

                    time.sleep(1)   ## **change when perfected
                    return x,y;
                else:
                    index=index +1


########
def Precision_Land(vehicle,duration,DesiredSig):
    print("Precision Land Function:")
    while True:
        spd = speed
        duration = duration
        Xcenter = 160
        Ycenter = 120
        HB = 20
        x,y= GetBlocks(DesiredSig)

        if x< (Xcenter - HB):        #sets Xaxis based on Pixy coordinate
	        Xaxis= spd;
        elif x> (Xcenter + HB):
        	Xaxis=-spd;
        else:
        	Xaxis=0;
        	print('X is centered')
        
        
        if y< (Ycenter - HB):        #sets Yaxis based on Pixy coordinate
        	Yaxis=-spd;
        elif y> (Ycenter + HB):
        	Yaxis=spd;
        else:
        	Yaxis=0;
        	print('Y is centered')
        
        
        if Xaxis==0 and Yaxis==0:        #Exits if within hit box
        	print('all centered...SUCK IT KEVIN')
        
        
        print('*************************')
        send_ned_velocity(vehicle, Xaxis, Yaxis, 0, duration)
    
    time.sleep(1)

	
################################ Velocity ################################

def send_ned_velocity(vehicle,velocity_x, velocity_y, velocity_z, duration):
    
    #Move vehicle in direction based on specified velocity vectors.
        
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
       # time.sleep(.05)	# Modify to scale duration time 


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

'''
################################ BreadCrumb Function ################################


def BreadCrumb():
    for i in range(0,2):
        if i ==0:
            DesiredSig=10; #12 in octal or 10 in decimal
        elif i==1:
            DesiredSig=11; #13 in octal
        else i=2:
            DesiredSig=12; #14 octal
        print(i)
        print(DesiredSig)
        Centering(DesiredSig)
        condition_yaw(0)	# Reorient to North
        i=i+1

'''
############################### Auto North ##################################


def Auto_Yaw(vehicle):
	
	flag=0;
	while 1:
		direction = vehicle.heading
		if direction < 15 or direction > 345:
			flag =1
			print("North reached")
			return flag
			break
		elif direction < 180:
			direction = -1;
		else: 
			direction = 1;
		condition_yaw(vehicle, direction)

################################ Centering Function ################################

def Centering(vehicle,duration,DesiredSig):
	print("Centering Function:")
	flag=0;
	while True:
		spd = speed
		duration = duration
		Xcenter = 160
		Ycenter = 120
		HB = 30
		print('get blocks')
		x,y= GetBlocks(DesiredSig)
                
		if x< (Xcenter - HB):        #sets Xaxis based on Pixy coordinate
			Xaxis= spd;
		elif x> (Xcenter + HB):
			Xaxis=-spd;
		else:
			Xaxis=0;
			print('X is centered')
        
        
		if y< (Ycenter - HB):        #sets Yaxis based on Pixy coordinate
			Yaxis=-spd;
		elif y> (Ycenter + HB):
			Yaxis=spd;
		else:
			Yaxis=0;
			print('Y is centered')
            
            
		if Xaxis==0 and Yaxis==0:        #Exits if within hit box
			print('all centered...SUCK IT KEVIN')
			break;
			flag=1;
			return flag
		else:
			print('*************************')
			print("Xaxis:",Xaxis)
			print("Yaxis:",Yaxis)
			send_ned_velocity(vehicle, Xaxis, Yaxis, 0, duration)
			Auto_Yaw(vehicle)

    
        
	time.sleep(.5)
############################### Main ##################################


parser=argparse.ArgumentParser(description='parameters')
parser.add_argument('--speed',help='set speed')
parser.add_argument('--desiredAlt',help='set desiredAlt')
parser.add_argument('--duration',help='set duration')
parser.add_argument('--connect',help='set connection string i.e. 127.0.0.1:14550')
parser.add_argument('--desiredSig',help='set desiredSig')


args=parser.parse_args()
desiredAlt=float(args.desiredAlt)
desiredSig=float(args.desiredSig)
speed=float(args.speed)
duration=int(args.duration);
connection_string=args.connect

pixy_init()

vehicle=connect_it(connection_string)    #Connect to Pixhawk via Pi
Get_Parameters(vehicle)        #Get start up parameters of UAS
Arm_it(vehicle)        #arm the motors
time.sleep(3)

takeoff(vehicle,desiredAlt)
send_ned_velocity(vehicle,0, 0, 0, 1)#dummy movement command--initialize mov					ement
time.sleep(1)
flag =Auto_Yaw(vehicle)
time.sleep(2)


if flag ==1:
	flag=Centering(vehicle,duration,desiredSig) #center at first signature
	print(vehicle.heading)	
	land_it(vehicle)  #return to launch
	Precision_Land(vehicle,duration,desiredSig)
else:
	time.sleep(1)




