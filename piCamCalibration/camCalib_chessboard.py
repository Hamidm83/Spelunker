
import os
import math
import numpy
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

if __name__ == '__main__':
    camera = PiCamera()
    #camera.framerate = 32
    #camera.exposure_mode = 'auto'
    #camera.awb_mode = 'auto'
    camera.resolution = (640, 480)
    print('ex:',camera.exposure_mode)
    print('wb:',camera.awb_mode)
    print('rs:',camera.resolution)
    
    rawCapture = PiRGBArray(camera)
    time.sleep(0.1)
    numFrame = 0

    objpoints = [] # 3D point in real world space where chess squares are.  world_points = []
    imgpoints = [] # 2D point in image plane, determined by CV2
    CHESSBOARD_CORNERS_ROWCOUNT = 9
    CHESSBOARD_CORNERS_COLCOUNT = 6
    # Create arrays you'll use to store object points and image points from all images processed
    objp = numpy.zeros((CHESSBOARD_CORNERS_ROWCOUNT*CHESSBOARD_CORNERS_COLCOUNT,3), numpy.float32)
    objp[:,:2] = numpy.mgrid[0:CHESSBOARD_CORNERS_ROWCOUNT,0:CHESSBOARD_CORNERS_COLCOUNT].T.reshape(-1, 2)

    imageSize = None 

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        src = image
        image_output = src.copy()

        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        if not imageSize:
            imageSize = gray.shape[::-1]
        return_status, corners = cv2.findChessboardCorners(gray, (CHESSBOARD_CORNERS_ROWCOUNT,CHESSBOARD_CORNERS_COLCOUNT), None)
        if return_status == True and numFrame%3 == 0:
            objpoints.append(objp)
            corners_acc = cv2.cornerSubPix(image=gray, corners=corners, winSize=(11, 11), zeroZone=(-1, -1),criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)) # Last parameter is about termination critera
            imgpoints.append(corners_acc)    
            image_output = cv2.drawChessboardCorners(image_output, (CHESSBOARD_CORNERS_ROWCOUNT, CHESSBOARD_CORNERS_COLCOUNT), corners_acc, return_status)
        
        if(len(imgpoints)>0):
            image_output = cv2.putText(image_output, str(len(imgpoints)), (50,50), 1,2, (200,200,0), 2, cv2.LINE_AA) 
        cv2.imshow('Chessboard', image_output)
        key = cv2.waitKey(1)
        rawCapture.truncate(0)
        numFrame += 1

        if key ==13:
            fineName = 'img_'+str(numFrame)+'.jpg'
            print(fineName)
            cv2.imwrite(fineName,image_output)
        if key == 27 or len(imgpoints) == 100:
            break

    cv2.destroyAllWindows()

    if(len(imgpoints)>10):
        print('Start Calibration')
        calibration_status, camera_matrix, distortion_coefficient, rotation_vectors, translation_vectors = cv2.calibrateCamera(objectPoints=objpoints, imagePoints=imgpoints, imageSize=imageSize, cameraMatrix=None, distCoeffs=None)

        print(calibration_status)
        print(camera_matrix)

        if(calibration_status < 1.0):
            calibration_file = cv2.FileStorage('calibration.yaml', cv2.FILE_STORAGE_WRITE)
            calibration_file.write("camera_shape", imageSize)
            calibration_file.write("camera_matrix", camera_matrix)
            calibration_file.write("distortion_coefficients ", distortion_coefficient)
            calibration_file.release()
