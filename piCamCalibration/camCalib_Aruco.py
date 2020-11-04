import numpy
import cv2
from cv2 import aruco
import pickle
import glob

# from userDefine file
CHARUCOBOARD_ROWCOUNT = 7
CHARUCOBOARD_COLCOUNT = 5 
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_100)

CHARUCO_BOARD = aruco.CharucoBoard_create(
        squaresX=CHARUCOBOARD_COLCOUNT,
        squaresY=CHARUCOBOARD_ROWCOUNT,
        squareLength=0.04,
        markerLength=0.02,
        dictionary=ARUCO_DICT)
#def main():
#    dictionary_bits = int(sys.argv[1])
#    dictionary_length = int(sys.argv[2])
#    marker_size_in_mm = int(sys.argv[3])

#    if not os.path.exists(f'./DICT_{dictionary_bits}X{dictionary_bits}_{dictionary_length}'):
#        os.mkdir(f'./DICT_{dictionary_bits}X{dictionary_bits}_{dictionary_length}')

#    aruco_dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, f'DICT_{dictionary_bits}X{dictionary_bits}_{dictionary_length}'))

#    for i in range(0, dictionary_length):
#        aruco_marker = cv2.aruco.drawMarker(aruco_dictionary, i, mm_to_pixel(marker_size_in_mm))
#        cv2.imwrite(f'./DICT_{dictionary_bits}X{dictionary_bits}_{dictionary_length}/{i}_HAMID.png', aruco_marker)

#    return


#if __name__ == '__main__':
#    if len(sys.argv) == 4:
#        main()
#    else:
#        print(f'Usage: {sys.argv[0]} <dictionary_bits: Integer: 4, 5, 6, 7> <dictionary_length: Integer: 50, 100, 250, 1000> <marker_size_in_mm: Integer>')

#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
#board = cv2.aruco.CharucoBoard_create(3,3,.025,.0125,dictionary)
#img = board.draw((200*3,200*3))

##Dump the calibration board to a file
#cv2.imwrite('charuco.png',img)


##Start capturing images for calibration
##cap = cv2.VideoCapture(0)

#allCorners = []
#allIds = []
#decimator = 0
#for i in range(300):

#    #ret,frame = cap.read()
#    frame = cv2.imread('img_33.jpg')
#    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#    res = cv2.aruco.detectMarkers(gray,dictionary)

#    if len(res[0])>0:
#        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
#        if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
#            allCorners.append(res2[1])
#            allIds.append(res2[2])

#        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])

#    cv2.imshow('frame',gray)
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break
#    decimator+=1

#imsize = gray.shape

##Calibration fails for lots of reasons. Release the video if we do
#try:
#    cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
#except:
#    cap.release()

#cap.release()
#cv2.destroyAllWindows()
#import cv2 as cv
#import numpy as np
#from matplotlib import pyplot as plt  # Matplotlib for plotting

#s = cv.FileStorage('response_P.xml', cv.FileStorage_WRITE)

## Loading exposure images into a list
#img_fn = ["img_1000.bmp", "img_11000.bmp", "img_21000.bmp", "img_31000.bmp","img_41000.bmp", "img_51000.bmp", "img_61000.bmp"]
#img_list = [cv.imread(fn) for fn in img_fn]
#exposure_times = np.array([1000.0, 11000, 21000, 31000,41000,51000,61000], dtype=np.float32)
##exposure_times = np.array([15.0, 2.5, 0.25, 0.0333], dtype=np.float32)

## Estimate camera response function (CRF)
#cal_debevec = cv.createCalibrateDebevec()
#crf_debevec = cal_debevec.process(img_list, exposure_times)
#print(crf_debevec )
#x,y,z = cv.split(crf_debevec)
#s.write('x', x)
#s.write('y', y)
#s.write('z', z)

##cal_robertson = cv.createCalibrateRobertson()
##crf_robertson = cal_robertson.process(img_list, times=exposure_times)
##print(crf_robertson)

#s.release()


import time
import cv2
import cv2.aruco as A
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
board = cv2.aruco.CharucoBoard_create(3,3,.025,.0125,dictionary)
img = board.draw((200*3,200*3))

#Dump the calibration board to a file
cv2.imwrite('charuco.png',img)


#Start capturing images for calibration
#cap = cv2.VideoCapture(0)

allCorners = []
allIds = []
decimator = 0
for i in range(300):

    #ret,frame = cap.read()
    frame = cv2.imread('img_33.jpg')
    cv2.imshow('frame',frame)
    cv2.waitKey(1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    res = cv2.aruco.detectMarkers(gray,dictionary)

    if len(res[0])>0:
        res2 = cv2.aruco.interpolateCornersCharuco(res[0],res[1],gray,board)
        if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%3==0:
            allCorners.append(res2[1])
            allIds.append(res2[2])

        cv2.aruco.drawDetectedMarkers(gray,res[0],res[1])

    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    decimator+=1

imsize = gray.shape

#Calibration fails for lots of reasons. Release the video if we do
try:
    cal = cv2.aruco.calibrateCameraCharuco(allCorners,allIds,board,imsize,None,None)
except:
    cap.release()

cap.release()
cv2.destroyAllWindows()
