import socketio
import base64
import numpy as np
import json
import estimatePose
import sys
import time
import os
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
from cv2 import aruco
import VampireApi as vmp

from time import sleep
import board
import busio
import RPi.GPIO as GPIO
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from Adafruit_CCS811 import Adafruit_CCS811



sio = socketio.Client()

    #Alarm
def checkSelfAlarm(temp ,tvoc ,co2):
    if temp > 40 or tvoc > 500 or co2 > 1000:
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(18,GPIO.OUT)
        print ("LED on")
        GPIO.output(18,GPIO.HIGH)
        time.sleep(1)
        GPIO.output(18,GPIO.LOW)
        print ("Acceleration = %.2f m/s^2" % acc)
   
    #Accelerometer 
def sensor_data_acc():
    i2c = busio.I2C(board.SCL, board.SDA)
    accl_sensor = LSM6DSOX(i2c)
    acc1 = 0
    while True:
        x = accl_sensor.acceleration[0]
        y = accl_sensor.acceleration[1]
        z = accl_sensor.acceleration[2]
        acc = math.sqrt(x*x + y*y + z*z)
        sio.emit('my_message',{'acc':acc1})
        if acc > 20:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(18,GPIO.OUT)
            GPIO.output(18,GPIO.HIGH)
            time.sleep(2)
            GPIO.output(18,GPIO.LOW)
        sio.sleep(1)


     #Gas sensor
def sensor_data_gas():
    ccs =  Adafruit_CCS811()
    while not ccs.available():
        pass
    temp = ccs.calculateTemperature()
    ccs.tempOffset = temp - 25.0
    while ccs.available():
        temp = ccs.calculateTemperature()
        temp = round(temp,2)
        co2 = ccs.geteCO2()
        tvoc = ccs.getTVOC() # Total Volatile Organic Compounds (TVOCs)
        if not ccs.readData():
          #print ("CO2: ", co2, "ppm, TVOC: ", tvoc, " temp:", (round(temp,2)))
            sio.emit('my_message',{'co2':co2})
            sio.emit('my_message',{'tvoc':tvoc})
            sio.emit('my_message',{'temp':temp})
            sio.sleep(1)
        checkSelfAlarm(temp ,tvoc ,co2)
#def sensor_data_gas():
#    val = 1
#    while True:
#        val = val + 5
#        sio.emit('my_message',{'co2':val})
#        val = val + 10
#        sio.emit('my_message',{'temp':val})
#        sio.sleep(2)

def aprilTagPositioning():
    CHARUCOBOARD_X = 10
    CHARUCOBOARD_Y = 7
    SQUARE_LENGTH = 33
    MARKER_LENGTH = 24.5
    showMarker = False
    isVampire = False
    ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    chruco_board = cv2.aruco.CharucoBoard_create(
        squaresX=CHARUCOBOARD_X,
        squaresY=CHARUCOBOARD_Y,
        squareLength=SQUARE_LENGTH,
        markerLength=MARKER_LENGTH,
        dictionary=ARUCO_DICT)
    camera_matrix,dis_coeffs = estimatePose.readCalibrationFile('calibration.yaml')
    camera = PiCamera()
    camera.resolution = (640, 480)
    print('ex:',camera.exposure_mode)
    print('wb:',camera.awb_mode)
    print('rs:',camera.resolution)
    rawCapture = PiRGBArray(camera)
    time.sleep(0.1)
    numFrame = 0
    key = -1
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        if isVampire == True:
            image  = vmp.vamp_test_pyMat(image,0.7)
        src = image
        image_output = src.copy()
        gray_img= cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected_corners = cv2.aruco.detectMarkers(gray_img, ARUCO_DICT)
        if showMarker == True:
            image_output = cv2.aruco.drawDetectedMarkers(image_output, corners, ids,borderColor=(150, 0, 150))
        if np.all(ids != None): 
            rotation_vectors, translation_vectors, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, camera_matrix, dis_coeffs)           
            for rvec, tvec in zip(rotation_vectors, translation_vectors):
                    image_output = aruco.drawAxis(image_output, camera_matrix, dis_coeffs , rvec, tvec, SQUARE_LENGTH)
        #cv2.imshow('tracking', image_output)
        retval, buffer = cv2.imencode('.jpg', image_output)
        jpg_as_text = base64.b64encode(buffer)
        jpg_original = base64.b64decode(jpg_as_text)
        sio.emit('my_message',{'piCam':jpg_original})
        #key = cv2.waitKey(1)

        rawCapture.truncate(0)
        numFrame += 1
        if key == 118 :
            isVampire = not isVampire
            print('vampire', isVampire)
        sio.sleep(1)

@sio.event
def connect():
    print('connection established')
    sio.start_background_task(aprilTagPositioning)
    sio.start_background_task(sensor_data_gas)
    sio.start_background_task(sensor_data_acc)

@sio.event
def my_message(data):
    print('message received with ', data)
    sio.emit('my response', {'response': 'my response'})

@sio.event
def disconnect():
    print('disconnected from server')

#sio.connect('http://192.168.0.31:2607', headers={'device_id':'pi_1'}) #SirjanNAlbany
#sio.connect('http://192.168.43.119:12607', headers={'device_id':'pi_1'}) # HamidPhone Hotspot    IPv4 Address. . . . . . . . . . . : 192.168.43.119
sio.connect('http://172.16.13.44:12607', headers={'device_id':'pi_48'}) # Trimble Guest   IPv4 Address. . . . . . . . . . . : 172.16.13.44
sio.wait()