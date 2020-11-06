
import time
from time import sleep
import board
import busio
import math
import RPi.GPIO as GPIO
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from Adafruit_CCS811 import Adafruit_CCS811

i2c = busio.I2C(board.SCL, board.SDA)

sensor = LSM6DSOX(i2c)

ccs =  Adafruit_CCS811()

while not ccs.available():
    pass
temp = ccs.calculateTemperature()
ccs.tempOffset = temp - 25.0

while True:
    #Accelerometer 
    #print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (sensor.acceleration))
    #print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s" % (sensor.gyro))
    x = sensor.acceleration[0]
    y = sensor.acceleration[1]
    z = sensor.acceleration[2]
    acc = math.sqrt(x*x + y*y + z*z)
    print ("Acceleration = %.2f m/s^2" % acc)
    
    #Gas sensor
    if ccs.available():
        temp = ccs.calculateTemperature()
        
        co2 = ccs.geteCO2()
        tvoc = ccs.getTVOC() # Total Volatile Organic Compounds (TVOCs)
        
        if not ccs.readData():
          print ("CO2: ", co2, "ppm, TVOC: ", tvoc, " temp:", (round(temp,2)))

        else:
          print ("ERROR!")
          while(1):
            pass
    sleep(0.5)
    
    #Alarm
    if acc > 20 or temp > 40 or tvoc > 500 or co2 > 1000:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(18,GPIO.OUT)
        print ("LED on")
        GPIO.output(18,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(18,GPIO.LOW)
        print ("Acceleration = %.2f m/s^2" % acc)
    
    print("")
    time.sleep(0.5)
    
   
