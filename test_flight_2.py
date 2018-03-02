
from UAS_Functions import turnOffMotors, Arm_it, connect_it,Get_Parameters, takeoff,land_it
from dronekit import connect, VehicleMode #, LocationGlobalRelative
import time
import atexit
import RPi.GPIO as GPIO
from ctypes import *
import sys

vehicle=connect_it()    #Connect to Pixhawk via Pi
Get_Parameters()        #Get start up parameters of UAS
Arm_it(vehicle)        #arm the motors
wait(1)

desiredAlt=int(sys.args[1])

takeoff(desiredAlt)

wait(5)

speed=int(sys.args[1])
duration=int(sys.args[2]);

#velocity_x, velocity_y, velocity_z, duration

print("testing x direction")
send_ned_velocity(speed,0,0,duration)
waut(5)

print("testing y direction")
send_ned_velocity(0,speed,0,duration)
wait(5)

print("testing both axis")
send_ned_velocity(speed,speed,0,duration)
wait(5)

land_it

vehicle.armed=False
