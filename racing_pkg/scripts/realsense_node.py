#!/usr/bin/env python3

# from pyvicon_datastream import tools

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

" Set resolution and frame rate for data collection "
# set1: 848x480, 60fps
# set2: 1280x720, 30fps
# https://dev.intelrealsense.com/docs/high-speed-capture-mode-of-intel-realsense-depth-camera-d435

resolution_set = 2
if resolution_set == 1: 
    resolution_x, resolution_y, frame_rate = 848, 480, 60 
if resolution_set == 2: 
    resolution_x, resolution_y, frame_rate = 1280, 720, 30 
if resolution_set == 3: 
    resolution_x, resolution_y, frame_rate = 1024, 768, 30 
if resolution_set == 4: 
    resolution_x, resolution_y, frame_rate = 640, 480, 30 


class RealSence:
    def __init__(self):
        rospy.Rate(20)

        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.color, resolution_x, resolution_y, rs.format.bgr8, frame_rate)
        self.cfg.enable_stream(rs.stream.depth, resolution_x, resolution_y, rs.format.z16, frame_rate)
        self.profile = self.pipe.start(self.cfg)

        self.is_done = False

        " To test gate detection models "
        self.bridge = CvBridge()
        self.img_pub = rospy.Publisher("/camera_1/image_raw_1", Image, queue_size=10) 

    def get_image(self):
        " get image from Realsence camera "
        frame = self.pipe.wait_for_frames()
        color_frame = frame.get_color_frame()

        " convert images to numpy arrays "
        color_image = np.asanyarray(color_frame.get_data())

        " Prepare to publish to topic "
        imgmsg = self.bridge.cv2_to_imgmsg(np.uint8(color_image), "bgr8")
        self.img_pub.publish(imgmsg)

        " depth image aling to color frame "
        align = rs.align(rs.stream.color)
        frameset = align.process(frame)
        aligned_depth_frame = frameset.get_depth_frame()

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        if cv2.waitKey(1) == ord('q'):
            self.is_done = True

        return self.is_done
        
if __name__ == '__main__':
    rospy.init_node('data_collection', anonymous=True)

    node = RealSence()
    
    while not rospy.is_shutdown():
        is_done = node.get_image()
        if is_done:
            break
        
