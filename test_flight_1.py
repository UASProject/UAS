from UAS_Functions import Arm_it, connect_it,Get_Parameters#, takeoff,land_it
from dronekit import connect, VehicleMode #, LocationGlobalRelative
import time
import atexit
#import RPi.GPIO as GPIO
from ctypes import *
import sys

vehicle=connect_it()    #Connect to Pixhawk via Pi
Get_Parameters(vehicle)        #Get start up parameters of UAS
Arm_it(vehicle)        #arm the motors
time.sleep(5)

desiredAlt=5
'''
takeoff(vehicle,desiredAlt)

time.sleep(3)

land_it(vehicle)

vehicle.armed=False
'''
