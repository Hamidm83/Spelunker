import sys
import time
import numpy
import os
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from cv2 import aruco
import VampireApi as vmp


def readCalibrationFile(filePath):
    cv_file = cv2.FileStorage(filePath, cv2.FileStorage_READ)

    calib_value = cv_file.getNode('calib_value').real()
    #image_size = cv_file.getNode('image_sicamera ze')
    camera_matrix = cv_file.getNode('camera_matrix').mat()
    dist_matrix = cv_file.getNode('distortion_coefficients').mat()
    
    print(calib_value)
    cv_file.release()
    return camera_matrix, dist_matrix

def posEstimate_solvePnP():
        # Find the rotation and translation vectors.
        _,rvecs, tvecs, inliers = cv2.solvePnPRansac(objp, corners2, mtx, dist)
        # project 3D points to image plane
        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
        return

def inverse_pose(rotation_vector, translation_vector):
    rotation_matrix, jacobian = cv2.Rodrigues(rotation_vector)
    rotation_matrix = numpy.matrix(rotation_matrix).T

    inverse_rotation_vector, jacobian = cv2.Rodrigues(rotation_matrix)
    inverse_translation_vector = numpy.dot(-rotation_matrix, numpy.matrix(translation_vector))

    return inverse_rotation_vector, inverse_translation_vector


def relative_pose(rotation_vector_parent, translation_vector_parent, rotation_vector_child, translation_vector_child):
    rotation_vector_parent, translation_vector_parent = rotation_vector_parent.reshape((3, 1)), translation_vector_parent.reshape((3, 1))
    rotation_vector_child, translation_vector_child = rotation_vector_child.reshape((3, 1)), translation_vector_child.reshape((3, 1))

    inverse_rotation_vector_child, inverse_translation_vector_child = inverse_pose(rotation_vector_child, translation_vector_child)

    composed_matrix = cv2.composeRT(rotation_vector_parent, translation_vector_parent, inverse_rotation_vector_child, inverse_translation_vector_child)
    composed_rotation_vector = composed_matrix[0]
    composed_translation_vector = composed_matrix[1]

    composed_rotation_vector.reshape((3, 1))
    composed_translation_vector. reshape((3, 1))

    return composed_rotation_vector, composed_translation_vector


def main():
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

    camera_matrix,dis_coeffs = readCalibrationFile('calibration.yaml')

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
        #corners, ids, rejected_corners = cv2.aruco.detectMarkers(gray_img, ARUCO_DICT, parameters=aruco_parameters,cameraMatrix=camera_matrix,distCoeff=dis_coeffs)
        corners, ids, rejected_corners = cv2.aruco.detectMarkers(gray_img, ARUCO_DICT)
        
        if showMarker == True:
            image_output = cv2.aruco.drawDetectedMarkers(image_output, corners, ids,borderColor=(150, 0, 150))
   
        if numpy.all(ids != None): 
            rotation_vectors, translation_vectors, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, camera_matrix, dis_coeffs)           
            for rvec, tvec in zip(rotation_vectors, translation_vectors):
                    image_output = aruco.drawAxis(image_output, camera_matrix, dis_coeffs , rvec, tvec, SQUARE_LENGTH)

        cv2.imshow('tracking', image_output)
        key = cv2.waitKey(1)
        rawCapture.truncate(0)
        numFrame += 1

        if key == 118 :
            isVampire = not isVampire
            print('vampire', isVampire)
        if key == 27:
            break

    cv2.destroyAllWindows()
    return


if __name__ == '__main__':
    main()