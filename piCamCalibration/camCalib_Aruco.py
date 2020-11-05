import numpy
import os
import math
import numpy
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from cv2 import aruco



def mm_to_pixel(mm):
    return int((mm * 72) / 25.4)

def generate_marker(dictionary_length, aruco_dictionary):
    marker_size_in_mm = 700
    if not os.path.exists(f'./DICT_MARKS'):
        os.mkdir(f'./DICT_MARKS')
    for i in range(0, dictionary_length):
        aruco_marker = cv2.aruco.drawMarker(aruco_dictionary, i, mm_to_pixel(marker_size_in_mm))
        cv2.imwrite(f'./DICT_MARK/t_{i}.png', aruco_marker)
        
    return

def undistortion(img,mtx,dist):
    ## undistort
        h,  w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        return dst

def calculateCalibration(charucoCorners,charucoIds,ch_board,imageSize):
        print('Start Calibration')
        retVal, camera_matrix, distortion_coefficient, rotation_vectors, translation_vectors = cv2.aruco.calibrateCameraCharuco(
        charucoCorners,
        charucoIds,
        ch_board,
        imageSize,None, None)
        print(retVal)
        print(camera_matrix)

        if(retVal > 0.0):
            calibration_file = cv2.FileStorage('calibration.yaml', cv2.FILE_STORAGE_WRITE)
            calibration_file.write("calib_value", retVal)
            calibration_file.write("image_size", imageSize)
            calibration_file.write("camera_matrix", camera_matrix)
            calibration_file.write("distortion_coefficients ", distortion_coefficient)
            calibration_file.release()
        return retVal, camera_matrix, distortion_coefficient, rotation_vectors, translation_vectors




if __name__ == '__main__':
## todo reading from userDefined.xml
#<BoardSize_Width>  10
#<BoardSize_Height> 7
    CHARUCOBOARD_X = 10
    CHARUCOBOARD_Y = 7
    SQUARE_LENGTH = 33
    MARKER_LENGTH = 24.5
    showMarker = False
    showRejected = False
    showCharacoCorners = True


    camera = PiCamera()
    #camera.resolution = (640, 480)
    print('ex:',camera.exposure_mode,'\twb:',camera.awb_mode)
    print('rs:',camera.resolution)
    rawCapture = PiRGBArray(camera)
    time.sleep(0.1)
    numFrame = 0

    ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    chruco_board = cv2.aruco.CharucoBoard_create(
        squaresX=CHARUCOBOARD_X,
        squaresY=CHARUCOBOARD_Y,
        squareLength=SQUARE_LENGTH,
        markerLength=MARKER_LENGTH,
        dictionary=ARUCO_DICT)

## CHECK
    #board = cv2.aruco.Board(chruco_board)
## /CHECK
    objPts = chruco_board.chessboardCorners;
    objectPoints = []
    allCharucoCorners = []
    allCharucoIds =[]
    corners_all = [] 
    ids_all = [] 
    image_size = None 
  

    isCalibrated = False
    cam_mat = None
    dis_coef = None
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        src = image
        image_output = src.copy()

        if(isCalibrated == True):
            image_output = cv2.putText(image_output, 'undistorted', (50,50), 1,2, (0,200,200), 2, cv2.LINE_AA) 
            image_output = undistortion(image_output, cam_mat,dis_coef)
        else:
            gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
            if not image_size:
                image_size = gray.shape[::-1]

            corners, ids, rejectedCorners = cv2.aruco.detectMarkers(image=gray,dictionary=ARUCO_DICT)
            if len(corners) > 0 and numFrame%7 == 0:
                corners, ids, rejectedCorners, recoveredIdxs=cv2.aruco.refineDetectedMarkers(
            	image, chruco_board, corners, ids, rejectedCorners)
                charuco_res, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=chruco_board)
                
                if showMarker == True:
                  image_output = cv2.aruco.drawDetectedMarkers(image_output, corners, ids)
                if showRejected == True and len(rejectedCorners) > 0:
                    image_output = cv2.aruco.drawDetectedMarkers(image=image_output, corners=rejectedCorners)

                #if ids is not None and corners is not None and len(ids) > 0 and len(corners) > 0 and len(corners) == len(ids):
                #    if len(ids) == len(chruco_board.ids):
                #        ret, cameraMatrix, distCoeffs, _, _ = cv2.calibrateCamera(
                #        objectPoints=board.objPoints,
                #        imagePoints=corners,
                #        imageSize=image_size, 
                #        cameraMatrix=None,
                #        distCoeffs=None)

                if charuco_res > 12:
                    corners_acc = cv2.cornerSubPix(image=gray, corners=charuco_corners, winSize=(11, 11), zeroZone=(-1, -1),criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                    objectPoints.append(objPts)
                    allCharucoCorners.append(corners_acc)    
                    allCharucoIds.append(charuco_ids)
                    corners_all.append(corners)
                    ids_all.append(ids)

                    if showCharacoCorners == True and len(corners_acc) > 0:
                        image_output = cv2.aruco.drawDetectedCornersCharuco(image_output, corners_acc, charuco_ids, (255, 0, 0))
        


            if(len(allCharucoCorners)>0):
                image_output = cv2.putText(image_output, str(len(allCharucoCorners)), (50,50), 1,2, (200,200,0), 2, cv2.LINE_AA) 
            
        cv2.imshow('Charuco camera calibration', image_output)
        key = cv2.waitKey(1)
        rawCapture.truncate(0)
        numFrame += 1

        if isCalibrated == False and (key == 13 or len(allCharucoCorners) == 100):
            cv2.destroyAllWindows()
            ret_val, cam_mat, dis_coef, rot_vectors, translation_vectors = calculateCalibration(allCharucoCorners,allCharucoIds,chruco_board,image_size)
            print(ret_val)
            isCalibrated = True

        if key == 27:
            break

    cv2.destroyAllWindows()
