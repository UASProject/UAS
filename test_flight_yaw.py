from UAS_Functions import Arm_it, connect_it,Get_Parameters, takeoff,land_it,send_ned_velocity
from dronekit import connect, VehicleMode #, LocationGlobalRelative
import time, argparse
from pymavlink import mavutil
import atexit
from BreadCrumb import Auto_Yaw
#import RPi.GPIO as GPIO
from ctypes import *
import sys

parser=argparse.ArgumentParser(description='parameters')
parser.add_argument('--desiredAlt',help='set desired altitude')
parser.add_argument('--connect',help='set connection string i.e. 127.0.0.1:14550')

args=parser.parse_args()
desiredAlt=float(args.desiredAlt)
connection_string=args.connect

print'***************'

print'DesiredAlt: ',desiredAlt


vehicle=connect_it(connection_string)    #Connect to Pixhawk via Pi
Get_Parameters(vehicle)        #Get start up parameters of UAS

Arm_it(vehicle)        #arm the motors
time.sleep(3)


takeoff(vehicle,desiredAlt)

time.sleep(3)
print(vehicle.heading)

send_ned_velocity(vehicle,0,0,0,1) #"move" to allow for yawing

print('centering north')
flag=Auto_Yaw(vehicle,0)
print("reached due North")
time.sleep(3)

if flag ==1:
	land_it(vehicle)
else:
	time.sleep(1)



