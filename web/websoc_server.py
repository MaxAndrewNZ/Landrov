import traceback,sys
import asyncio
import websockets
import numpy as np
import zmq,pickle


context = zmq.Context()
control_socket = context.socket(zmq.PUB)
control_socket.connect("tcp://localhost:5556")

print('-1-')

async def hello(websocket, path):
    while 1:
        asyncio.sleep(0.001)
        datastr = await websocket.recv()
        try:
            if 0:
                iters,wx,wy,px,py,ang,f=map(eval,datastr.split())
                isleft=px/wx<0.5
                print('{} {:5.2f}'.format(\
                    'L' if isleft else 'R',\
                    f*np.sin(np.deg2rad(ang))\
                    ,end='\r'))
            else:
                iters,lf,la,rf,ra=map(eval,datastr.split())
                lf=np.clip(lf,-2,2)/2*np.sign(np.sin(np.deg2rad(la)))
                rf=np.clip(rf,-2,2)/2*np.sign(np.sin(np.deg2rad(ra)))
                control_socket.send_multipart([b'motor_over',pickle.dumps((lf,rf))])
                print('{:5} {:4.2f} {:4.2f} {:4.2f} {:4.2f}'.format(iters,lf,la,rf,ra),end='\r')
        except:
            print('fail parse',end='\r')
            traceback.print_exc(file=sys.stdout)

        #print("< {}".format(name))

    #greeting = "Hello {}!".format(name)
    #await websocket.send(greeting)
    #print("> {}".format(greeting))

start_server = websockets.serve(hello, '0.0.0.0',9998 )

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()

