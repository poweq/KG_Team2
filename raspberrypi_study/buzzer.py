import RPi.GPIO as GPIO
import time

BUZZER = 12
S1 = 5
S2 = 13
S3 = 19

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER, GPIO.OUT)
GPIO.setup(S1,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(S2,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(S3,GPIO.IN,pull_up_down=GPIO.PUD_DOWN)

p = GPIO.PWM(BUZZER, 261)
p.stop()

old_1 = 0
new_1 = 0
old_2 = 0
new_2 = 0
old_3 = 0
new_3 = 0

try:
    while True:
        new_1 = GPIO.input(S1)
        new_2 = GPIO.input(S2)
        new_3 = GPIO.input(S3)  
        
        if new_1 != old_1:
            old_1 = new_1
            
            if new_1 == 1:
                p.start(50)
                p.ChangeFrequency(261)
                time.sleep(0.1)
                
                p.stop()
                time.sleep(0.1)
                
            time.sleep(0.1)
            
        elif new_2 != old_2:
              old_2 = new_2
            
              if new_2 == 1:
                  p.start(50)
                  p.ChangeFrequency(293)
                  time.sleep(0.1)
                
                  p.stop()
                  time.sleep(0.1)
                
              time.sleep(0.1)
            
        elif new_3 != old_3:
              old_3 = new_3
            
              if new_3 == 1:
                  p.start(50)
                  p.ChangeFrequency(329)
                  time.sleep(0.1)
                
                  p.stop()
                  time.sleep(0.1)
                
              time.sleep(0.1)


except KeyboardInterrupt:
    pass

p.stop()
GPIO.cleanup()