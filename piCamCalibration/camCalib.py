
import os
import math
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

if __name__ == '__main__':
    print("start ")
    camera = PiCamera()
    camera.framerate = 32
    camera.exposure_mode = 'auto'
    camera.awb_mode = 'auto'
    rawCapture = PiRGBArray(camera)
    time.sleep(0.1)
    numFrame = 0
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        src = image
        image_output = src.copy()

        cv2.imshow('input',image_output)
        key = cv2.waitKey(1)
        rawCapture.truncate(0)
        numFrame += 1

        if key ==13:
            fineName = 'img_'+str(numFrame)+'.jpg'
            print(fineName)
            cv2.imwrite(fineName,image_output)
        if key == 27 or numFrame == 10001:
            break