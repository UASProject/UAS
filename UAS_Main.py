from UAS_Functions import lightbulb_detach, lightbulb_attatch, turnOffMotors,servo_close, Arm_it, connect_it,Get_Parameters
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
from dronekit import connect, VehicleMode #, LocationGlobalRelative
#from BreadCrumb import BreadCrumb,Centering,GetBlocks
import time
import atexit
import RPi.GPIO as GPIO
from pixy import *
from ctypes import *

##################### Initial System Start up ######################
#pixy_init()             #Initialize Pixy Cam
#vehicle=connect_it()    #Connect to Pixhawk via Pi
#Get_Parameters()        #Get start up parameters of UAS
#Arm_it(vehicle)        #arm the motors

##################### Flight Functions ######################

#takeoff(DesiredAlt)     #take off
#BreadCrumb()            #trail Navigation
#BaseOps()              #Base function







'''
#arm motors and takeoff to set altitude

takeoff(aTargetAltitude)
vehicle.airspeed = 3 #set default speed
'''



#servo_close()

lightbulb_attatch()
lightbulb_detach()

