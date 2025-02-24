import gpiozero as gpio
import board
import time

motorBR = gpio.Motor(0,1,2,True,None)

SpeedTrck = 0

while(SpeedTrck > 0):
    motorBR.forward(SpeedTrck/1023) #setspeed forward
    time.sleep(0.1)
    motorBR.backward(SpeedTrck/1023) #setspeed forward
    time.sleep(0.1)

SpeedTrck += 1
