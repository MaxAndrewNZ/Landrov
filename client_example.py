# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import cv2,time,zmq,pickle
import numpy as np



############ connecting to landrov ##################
context = zmq.Context()
sensor_socket = context.socket(zmq.SUB)
sensor_socket.connect("tcp://192.168.8.106:5557")
sensor_socket.setsockopt(zmq.SUBSCRIBE,b'depthimage') 
sensor_socket.setsockopt(zmq.SUBSCRIBE,b'rgbimage') 

control_socket = context.socket(zmq.PUB)
control_socket.connect("tcp://192.168.8.106:5556")
print('connected to landrov server')


############      main loop        ##################

start_time = time.time()
while 1:
    k=cv2.waitKey(20)
        #if sensor_socket.poll(0.001):

    if k!=-1:
        if k  == 27 or k == ord('q'):
            break
    if  len(zmq.select([sensor_socket],[],[],0)[0]):
        topic,buf = sensor_socket.recv_multipart()
        #print('got topic',topic)
        if topic == b'rgbimage':
            img = cv2.imdecode(np.fromstring(buf, dtype=np.uint8),cv2.IMREAD_COLOR)
            cv2.imshow('img',img)
        if topic == b'depthimage':
            depth =  cv2.imdecode(np.fromstring(buf, dtype=np.uint8),cv2.IMREAD_GRAYSCALE)  
            color_depth=cv2.applyColorMap(depth, cv2.COLORMAP_JET)
            cv2.imshow('depth',color_depth)

    tdiff= time.time()-start_time 
    
    fact = 0.6

    if tdiff < 1.0:  #first second moving forward
        cmd = (fact,fact)
    elif tdiff < 3.0: # turnning
        cmd = (fact,-fact)
    elif tdiff < 4.0: 
        cmd = (-fact,-fact) # moving backwards 
    else:
        cmd = (0.0,0.0) # stopping
    
    control_socket.send_multipart([b'motor',pickle.dumps(cmd,0)])


