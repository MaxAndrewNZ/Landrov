# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import serial
import time
import zmq,pickle


ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(4)

port = "5556"
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.bind("tcp://*:%s" % port)
socket.setsockopt(zmq.SUBSCRIBE,b'motor') 
socket.setsockopt(zmq.SUBSCRIBE,b'motor_over') 

print('limitting to 80\%')
limit=0.8

last_t_sent=0
leftTrackVel = 0.
rightTrackVel = 0.

override_time=0
override_limit=10 #10sec

def is_override():
    return time.time()-override_time<override_limit

while 1:
    while ser.in_waiting:
        print('driver out = ',ord(ser.read()))

    if socket.poll(0.001): 
        topic, msg = socket.recv_multipart() 
        #print('topic=',topic)
        data = pickle.loads(msg)
        if topic==b'motor':
            if not is_override(): #not in overrride mode
                leftTrackVel,rightTrackVel=data
        if topic==b'motor_over':
            if data[0]>0.01 or data[1]>0.01: #moved stick enter overrride mode
                override_time=time.time()
            if is_override(): #10 sec in overrride mode
                leftTrackVel,rightTrackVel=data


    if time.time()-last_t_sent>0.1: #10hz
        left_value = int(round(191 + leftTrackVel*limit*63))
        right_value = int(round(63 + rightTrackVel*limit*63))
        #print("sent: {:3} {:3}".format(left_value,right_value ),end='\r')
        print("mode {}: {:3} {:3} {} {}".format(
            'O' if is_override() else ' ',
            left_value,right_value,leftTrackVel, rightTrackVel ),end='\r')
        ser.write(b'%c'%right_value)
        time.sleep(0.01)
        ser.write(b'%c'%left_value)
        last_t_sent=time.time()
        


