import RPi.GPIO as GPIO
import sys, argparse



parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--pulse',
                        help="pwm input for servo")
args = parser.parse_args()
                        
pulse = args.pulse
print(pulse)

def ServoTestOp(pulse):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(7,GPIO.OUT)
    
    for i in range(20):
        GPIO.output(7,1)
        time.sleep(pulse)
        GPIO.output(7,0)
        time.sleep(.1)
    
    for n in range(20):
        GPIO.output(7,1)
        time.sleep(0.0015)
        GPIO.output(7,0)
        time.sleep(.1)

    
    GPIO.cleanup()
############################


ServoTestOp
