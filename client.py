import socketio
import cv2

sio = socketio.Client()
def sensor_test():
    val = 1
    while True:
        val = val + 10
        sio.emit('my_message',{'hamid':val})
        sio.sleep(5)

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

sio.connect('http://localhost:5000', headers={'device_id':'pi_1'})
sio.wait()