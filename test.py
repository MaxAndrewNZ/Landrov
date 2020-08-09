# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
###############################################
##      Open CV and Numpy integration        ##
###############################################
print('-1-')

import pyrealsense2 as rs
print('-2-')
import numpy as np
import cv2
import pickle

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
sx=1280//2
sy=720//2
fps=30
#dest_sink='webmmux ! tcpserversink host=0.0.0.0 port=8888 sync=false'
#st-launch-1.0 videotestsrc ! x264enc noise-reduction=10000 speed-preset=ultrafast tune=zerolatency ! tcpserversink host=0.0.0.0 port=8888
dest_sink='tcpserversink host=0.0.0.0 port=8888'
#dest_sink='webmmux ! tcpserversink host=0.0.0.0 port=8888 sync=false'
#dest_sink='tcpserversink host=192.168.1.27 port=5000 sync=false'
config.enable_stream(rs.stream.depth, sx, sy, rs.format.z16, fps)
config.enable_stream(rs.stream.color, sx, sy, rs.format.bgr8, fps)

# Start streaming
cfg=pipeline.start(config)

profile_depth = cfg.get_stream(rs.stream.depth)
profile_rgb = cfg.get_stream(rs.stream.color)
intr_depth = profile_depth.as_video_stream_profile().get_intrinsics()
intr_rgb = profile_rgb.as_video_stream_profile().get_intrinsics()
print('intr_depth=',intr_depth)
print('intr_rgb=',intr_rgb)
print('extrinsics color2depth=',profile_rgb.get_extrinsics_to(profile_depth))
print('extrinsics depth2color=',profile_depth.get_extrinsics_to(profile_rgb))

align_to = rs.stream.color
align = rs.align(align_to)


if 0:
#'rtph264pay config-interval=1 pt=96 !'
#'x264enc noise-reduction=10000 speed-preset=ultrafast tune=zerolatency ! '
    out = cv2.VideoWriter('appsrc ! videoconvert ! '
                          'x264enc tune="zerolatency" ! '
                          + dest_sink,
                          0, fps, (sx*2, sy))

if 0:
    #send: gst-launch-1.0 videotestsrc horizontal-speed=5 ! x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! udpsink host=132.181.13.188 port=5000
    #recive: gst-launch-1.0 -v udpsrc port=5000 caps = "application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! decodebin ! videoconvert ! autovideosink

    dest_sink=' ! udpsink host=132.181.13.188 port=5000'
    out = cv2.VideoWriter('appsrc ! videoconvert ! '
                    'x264enc tune=zerolatency bitrate=1500 speed-preset=superfast ! rtph264pay '
                    + dest_sink,
                    0, fps, (sx*2, sy))

if 1:
    #recive ffplay -i tcp://132.181.13.180:8888

    out = cv2.VideoWriter('appsrc ! videoconvert ! '
                      'jpegenc ! '
                      + dest_sink,
                      0, fps, (sx*2, sy))
##
#fourcc = cv2.VideoWriter_fourcc(*'MJPG') 
#out = cv2.VideoWriter('appsrc ! videoconvert ! x264enc noise-reduction=10000 speed-preset=ultrafast tune=zerolatency !'
#	+dest_sink,cv2.CAP_GSTREAMER,fourcc,30.0,(sx*2,sy))

if not out.isOpened():
        print('Error openning stream')
#import ipdb;ipdb.set_trace()
cnt=0
try:
    while True:
        cnt+=1

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        

        #depth_frame = frames.get_depth_frame()
        #color_frame = frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        

        #import ipdb;ipdb.set_trace()
        if not depth_frame or not color_frame:
            continue
        print(cnt,end='\r')

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        out.write(images)

        # Show images
        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RealSense', images)
        #k=cv2.waitKey(1)
        #if k==ord('q'):
        #    break
        #if k==ord('m'):
        #    xx=500
        #    yy=300
        #    print(depth_frame.get_distance(xx,yy),depth_image[yy,xx])

        #f k==ord('s'):
        #    cv2.imwrite('data/rgb.png',color_image)
        #    pickle.dump(depth_image,open('data/depth.pkl','wb'),-1)
        

finally:

    # Stop streaming
    pipeline.stop()
