from UAS_Functions import lightbulb_detach, turnOffMotors,servo_close,GetBlocks,Centering #arm_and_takeoff
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
#from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import atexit
import RPi.GPIO as GPIO
from pixy import *
from ctypes import *

#connect to PI through serial
vehicle = connect(/dev/ttyAMA0, wait_ready=True)

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

# Initialize Pixy Interpreter thread #
pixy_init()
global Xaxis, Yaxis

#Centering()


'''
import argparse
parser = argparse.ArgumentParser(description='Start up command for Vehicle')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
'''

# Connect to the Vehicle
'''
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
'''
#arm motors and takeoff to set altitude
'''
#arm_and_takeoff(aTargetAltitude)
vehicle.airspeed = 3 #set default speed
'''
#get blocks data from pixy
'''
x,y=GetBlocks()
print(x)
print(y)



'''

#servo_close()

#lightbulb_detach()

