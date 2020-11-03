
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
    camera.resolution = (512, 512)

    rawCapture = PiRGBArray(camera)
    time.sleep(0.1)
    numFrame = 0

    CHESSBOARD_CORNERS_ROWCOUNT = 9
    CHESSBOARD_CORNERS_COLCOUNT = 6
#objpoints = [] # 3D point in real world space where chess squares are
    imgpoints = [] # 2D point in image plane, determined by CV2

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        src = image
        image_output = src.copy()

        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        board, corners = cv2.findChessboardCorners(gray, (CHESSBOARD_CORNERS_ROWCOUNT,CHESSBOARD_CORNERS_COLCOUNT), None)
        if board == True:
        # Enhance corner accuracy with cornerSubPix
            corners_acc = cv2.cornerSubPix(image=gray, corners=corners, winSize=(11, 11), zeroZone=(-1, -1),criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)) # Last parameter is about termination critera
            imgpoints.append(corners_acc)    
            image_output = cv2.drawChessboardCorners(image_output, (CHESSBOARD_CORNERS_ROWCOUNT, CHESSBOARD_CORNERS_COLCOUNT), corners_acc, board)
        
            cv2.imshow('Chessboard', image_output)
        else:
            cv2.imshow('Chessboard',gray)
        key = cv2.waitKey(1)
        rawCapture.truncate(0)
        numFrame += 1

        if key ==13:
            fineName = 'img_'+str(numFrame)+'.jpg'
            print(fineName)
            cv2.imwrite(fineName,image_output)
        if key == 27 or numFrame == 10001:
            break



#import numpy
#import cv2
#import pickle
#import glob


## Chessboard variables

#print(CHESSBOARD_CORNERS_ROWCOUNT,'++',CHESSBOARD_CORNERS_COLCOUNT)
## Theoretical object points for the chessboard we're calibrating against,
## These will come out like: 
##     (0, 0, 0), (1, 0, 0), ..., 
##     (CHESSBOARD_CORNERS_ROWCOUNT-1, CHESSBOARD_CORNERS_COLCOUNT-1, 0)
## Note that the Z value for all stays at 0, as this is a printed out 2D image
## And also that the max point is -1 of the max because we're zero-indexing
## The following line generates all the tuples needed at (0, 0, 0)
#objp = numpy.zeros((CHESSBOARD_CORNERS_ROWCOUNT*CHESSBOARD_CORNERS_COLCOUNT,3), numpy.float32)
## The following line fills the tuples just generated with their values (0, 0, 0), (1, 0, 0), ...
#objp[:,:2] = numpy.mgrid[0:CHESSBOARD_CORNERS_ROWCOUNT,0:CHESSBOARD_CORNERS_COLCOUNT].T.reshape(-1, 2)

#img = cv2.imread('img_91.jpg')
#gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#board, corners = cv2.findChessboardCorners(gray, (CHESSBOARD_CORNERS_ROWCOUNT,CHESSBOARD_CORNERS_COLCOUNT), None)
#if board == True:
#        # Add the points in 3D that we just discovered
#        objpoints.append(objp)
        
#        # Enhance corner accuracy with cornerSubPix
#        corners_acc = cv2.cornerSubPix(
#                image=gray, 
#                corners=corners, 
#                winSize=(11, 11), 
#                zeroZone=(-1, -1),
#                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)) # Last parameter is about termination critera
#        imgpoints.append(corners_acc)

#        imageSize = gray.shape[::-1]
    
#        # Draw the corners to a new image to show whoever is performing the calibration
#        # that the board was properly detected
#        img = cv2.drawChessboardCorners(img, (CHESSBOARD_CORNERS_ROWCOUNT, CHESSBOARD_CORNERS_COLCOUNT), corners_acc, board)
#        # Pause to display each image, waiting for key press
#        cv2.imshow('Chessboard', img)
#        cv2.waitKey(0)
#else:
#        print("Not able to detect a chessboard in image: {}".format(iname))

## Destroy any open CV windows
#cv2.destroyAllWindows()