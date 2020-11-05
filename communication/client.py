import socketio
import cv2
import base64
import numpy as np
import json


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

@sio.event
def connect():
    print('connection established')
    sio.start_background_task(sensor_test)

@sio.event
def my_message(data):
    print('message received with ', data)
    sio.emit('my response', {'response': 'my response'})

@sio.event
def disconnect():
    print('disconnected from server')

#sio.connect('http://192.168.0.31:2607', headers={'device_id':'pi_1'}) #SirjanNAlbany
sio.connect('http://192.168.43.119:12607', headers={'device_id':'pi_1'}) # HamidPhone Hotspot
sio.wait()