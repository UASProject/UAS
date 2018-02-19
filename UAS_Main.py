from UAS_Functions import lightbulb_detach, turnOffMotors,servo_close,GetBlocks,Centering, Arm_it, connect_it,Get_Parameters
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
from dronekit import connect, VehicleMode #, LocationGlobalRelative
import time
import atexit
import RPi.GPIO as GPIO
from pixy import *
from ctypes import *

# Initialize Pixy Interpreter thread #
pixy_init()

global Xaxis, Yaxis

vehicle=connect_it()
Get_Parameters()
#Arm_it(vehicle)

Centering()





'''
#arm motors and takeoff to set altitude

takeoff(aTargetAltitude)
vehicle.airspeed = 3 #set default speed
'''



#servo_close()

#lightbulb_detach()

