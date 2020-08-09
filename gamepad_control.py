# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import pygame,time,zmq,pickle
#pygame.init() ### =100%cpu

pygame.display.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
name = joystick.get_name()
print("Joystick name: {}".format(name))
axes = joystick.get_numaxes()
print( "Number of axes: {}".format(axes))
n_buttons = joystick.get_numbuttons()

clock = pygame.time.Clock()

port = "5556"
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("tcp://192.168.8.106:%s" % port)
print('connected to landrov server')


done = False
cnt=0
while not done:
    cnt+=1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
 
        if event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
            buttons = [joystick.get_button(i) for i in range(n_buttons)]
            print('buttons=',buttons)
        if event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")
        hold = joystick.get_hat(0)
        if abs(hold[0])>0 or abs(hold[1])>0:
            print('{} hold {}'.format(cnt,hold))


        axes_vals = []
        for i in range(axes):
            axis = joystick.get_axis(i)
            axes_vals.append(axis)
        if cnt%10==0:
            print('axes_vals=',','.join(['{:4.3f}'.format(i) for i in axes_vals]))
       
        cmd = (axes_vals[1],axes_vals[4])
        #socket.send(pickle.dumps(cmd,0)) 
        socket.send_multipart([b'motor',pickle.dumps(cmd,0)]) 
        #socket.send_multipart([config.topic_mixing,pickle.dumps((port,starboard,vertical),-1)])
        #print('{:> 5} P {:> 5.3f} S {:> 5.3f} V {:> 5.3f}'.format(cnt,port,starboard,vertical),end='\r')

    #pygame.time.wait(0)
    clock.tick(10)
pygame.quit()

