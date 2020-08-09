
import pygame,time,zmq,pickle,sys,cv2
import numpy as np
import pyrealsense2 as rs               
import open3d as o3d


clock = pygame.time.Clock()

############ connecting to landrov ##################
context = zmq.Context()
control_socket = context.socket(zmq.PUB)
control_socket.connect("tcp://192.168.8.106:5556")
print('connected to landrov server')

worldx = 50
worldy = 50
world = pygame.display.set_mode([worldx,worldy])

done = False
cnt = 0
leftMotor = 0.0 
rightMotor = 0.0
speed = 0.3
turningSpeed = 0.6

while not done:
    cnt += 1
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit(); sys.exit()
            done = True

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT or event.key == ord('a'):
                print('left')
                leftMotor -= turningSpeed
                rightMotor += turningSpeed
            if event.key == pygame.K_RIGHT or event.key == ord('d'):
                leftMotor += turningSpeed
                rightMotor -= turningSpeed
                print('right')
            if event.key == pygame.K_UP or event.key == ord('w'):
                leftMotor += speed
                rightMotor += speed
                print('forward')
            if event.key == pygame.K_DOWN or event.key == ord('s'):
                leftMotor -= speed
                rightMotor -= speed
                print('back')

        if event.type == pygame.KEYUP:
            if event.key == pygame.K_LEFT or event.key == ord('a'):
                leftMotor += turningSpeed
                rightMotor -= turningSpeed
                print('left stop')
            if event.key == pygame.K_RIGHT or event.key == ord('d'):
                leftMotor -= turningSpeed
                rightMotor += turningSpeed
                print('right stop')
            if event.key == pygame.K_UP or event.key == ord('w'):
                leftMotor -= speed
                rightMotor -= speed
                print('forwards stop')
            if event.key == pygame.K_DOWN or event.key == ord('s'):
                leftMotor += speed
                rightMotor += speed
                print('back stop')
            if event.key == ord('q'):
                leftMotor = 0.0
                rightMotor = 0.0
                pygame.quit()
                sys.exit()
                done = True 

        cmd = (leftMotor, rightMotor)
        control_socket.send_multipart([b'motor',pickle.dumps(cmd,0)]) 
    clock.tick(10)
    
pygame.quit()

