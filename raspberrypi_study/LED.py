import RPi.GPIO as GPIO
import time

LED_FR = 16
LED_FL = 22
LED_BR = 21
LED_BL = 28

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) #GPIO name gijun
GPIO.setup(LED_FL, GPIO.OUT)
GPIO.setup(LED_FR, GPIO.OUT)

try:
    while True:
        GPIO.output(LED_FL, GPIO.HIGH)
        GPIO.output(LED_FR, GPIO.LOW)
        time.sleep(1.0)

        GPIO.output(LED_FL, GPIO.LOW)
        GPIO.output(LED_FR, GPIO.HIGH)
        time.sleep(1.0)

except KeyboardInterrupt:
    pass

GPIO.cleanup()

