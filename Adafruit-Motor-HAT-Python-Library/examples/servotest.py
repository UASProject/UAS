import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(7,GPIO.OUT)

try:
	while True:
		for i in range(20):
			GPIO.output(7,1)
			time.sleep(0.0005)
			GPIO.output(7,0)
			time.sleep(.1)

		for f in range(20):
			GPIO.output(7,1)
			time.sleep(0.0025)
			GPIO.output(7,0)
			time.sleep(.1)

except KeyboardInterrupt:
	GPIO.cleanup()

