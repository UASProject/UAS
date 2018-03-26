
from UAS_Functions import Arm_it, connect_it,Get_Parameters, takeoff,land_it,send_ned_velocity
from dronekit import connect, VehicleMode #, LocationGlobalRelative
import RPi.GPIO as GPIO
import time
import atexit
from ctypes import *
import sys, argparse
from pymavlink import mavutil

parser=argparse.ArgumentParser(description='parameters')
parser.add_argument('--speed',help='set speed')
parser.add_argument('--desiredAlt',help='set desired altitude')
parser.add_argument('--duration',help='set duration')
parser.add_argument('--connect',help='set connection string i.e. 127.0.0.1:14550')

args=parser.parse_args()
desiredAlt=float(args.desiredAlt)
speed=float(args.speed)
duration=int(args.duration);
connection_string=args.connect

vehicle=connect_it(connection_string)    #Connect to Pixhawk via Pi
Get_Parameters(vehicle)        #Get start up parameters of UAS
Arm_it(vehicle)        #arm the motors
time.sleep(1)

'''
print'***************'
print'\n battery level:',vehicle.battery

print'DesiredAlt: ',desiredAlt
print'speed: ',speed
print'duration: ',duration

takeoff(vehicle,desiredAlt)

time.sleep(3)



#velocity_x, velocity_y, velocity_z, duration

print("testing x direction")
send_ned_velocity(vehicle,speed,0,0,duration)
time.sleep(3)

print("testing y direction")
send_ned_velocity(vehicle,0,speed,0,duration)
time.sleep(5)

print("testing -x direction")
send_ned_velocity(vehicle,-speed,0,0,duration)
time.sleep(3)

print("testing -y direction")
send_ned_velocity(vehicle,0,-speed,0,duration)
time.sleep(5)
'''

print("testing y direction")
send_ned_velocity(vehicle,0,speed,0,duration)
time.sleep(3)

print("testing -y direction")
send_ned_velocity(vehicle,0,-speed,0,duration)
time.sleep(3)

print("testing both axis")
send_ned_velocity(vehicle,speed,speed,0,duration)
time.sleep(3)

print("testing - both axis")
send_ned_velocity(vehicle,-speed,-speed,0,duration)
time.sleep(3)
'''
land_it(vehicle)

print'\n battery level:',vehicle.battery




'''
