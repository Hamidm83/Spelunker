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


sio = socketio.Client()
def sensor_test():
    val = 1
    while True:
        val = val + 10
        if val == 31:
            img = cv2.imread('/home/pi/10.jpg')
            retval, buffer = cv2.imencode('.jpg', img)
            jpg_as_text = base64.b64encode(buffer)
            jpg_original = base64.b64decode(jpg_as_text)
 #           jpg_as_np = np.frombuffer(jpg_original,dtype=np.uint8)            
            sio.emit('my_message',{'hamid':jpg_original})
        else:
            sio.emit('my_message',{'hamid':val})
        sio.sleep(2)

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
        sio.sleep(2)

@sio.event
def connect():
    print('connection established')
    sio.start_background_task(aprilTagPositioning)

@sio.event
def my_message(data):
    print('message received with ', data)
    sio.emit('my response', {'response': 'my response'})

@sio.event
def disconnect():
    print('disconnected from server')

#sio.connect('http://192.168.0.31:2607', headers={'device_id':'pi_1'}) #SirjanNAlbany
#sio.connect('http://192.168.43.119:12607', headers={'device_id':'pi_1'}) # HamidPhone Hotspot    IPv4 Address. . . . . . . . . . . : 192.168.43.119
sio.connect('http://172.16.13.44:12607', headers={'device_id':'pi_45'}) # Trimble Guest   IPv4 Address. . . . . . . . . . . : 172.16.13.44
sio.wait()