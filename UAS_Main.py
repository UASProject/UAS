from UAS_Functions import lightbulb_detach, turnOffMotors,servo_close,GetBlocks,Centering, Arm_it, connect_it
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
from dronekit import connect, VehicleMode #, LocationGlobalRelative
import time
import atexit
import RPi.GPIO as GPIO
from pixy import *
from ctypes import *


vehicle=connect_it()
Arm_it(vehicle)

'''
#connect to PI through serial
import argparse  
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)
'''
print "Get some vehicle attribute values:"
print " GPS: %s" % vehicle.gps_0
print " Battery: %s" % vehicle.battery
print " Last Heartbeat: %s" % vehicle.last_heartbeat
print " Is Armable?: %s" % vehicle.is_armable
print " System status: %s" % vehicle.system_status.state
print " Mode: %s" % vehicle.mode.name    # settable


'''
print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)


print("Arming motors")
# Copter should arm in GUIDED mode
VehicleMode('GUIDED')

vehicle.mode = VehicleMode("STABILIZE")
vehicle.armed = True

while not vehicle.armed:
    print(" Waiting for arming...")
    time.sleep(1)

   
'''

# Initialize Pixy Interpreter thread #
pixy_init()
global Xaxis, Yaxis

Centering()





'''
#arm motors and takeoff to set altitude

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

lightbulb_detach()

