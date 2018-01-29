from UAS_Functions import lightbulb_detach, turnOffMotors,servo_close,GetBlocks
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
import time
import atexit
import RPi.GPIO as GPIO
from pixy import *
from ctypes import *

# Initialize Pixy Interpreter thread #
pixy_init()

x,y=GetBlocks()
print(x)
print(y)
#servo_close()
#lightbulb_detach()

