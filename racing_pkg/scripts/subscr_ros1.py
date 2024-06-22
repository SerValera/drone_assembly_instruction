#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as Image
from geometry_msgs.msg import TransformStamped
import numpy as np
import time as t
import json
import os


class MinimalSubscriber():
    def __init__(self):

        self.sub1 = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.sub2 = rospy.Subscriber('/vicon/red/red', TransformStamped, self.vicon_callback)

        self.vic_pos = np.zeros([7])
        self.path = '/home/orangepi/Skoltech/dataset'
        self.all_poses = []

        self.count = 0
        self.size_of_dataset = 130

        self.t_start = t.time()
        self.capture_interval = 0.5 #sec

    def vicon_callback(self, msg):
        self.vic_pos[0] = msg.transform.translation.x
        self.vic_pos[1] = msg.transform.translation.y 
        self.vic_pos[2] = msg.transform.translation.z 
        self.vic_pos[3] = msg.transform.rotation.x
        self.vic_pos[4] = msg.transform.rotation.y
        self.vic_pos[5] = msg.transform.rotation.z
        self.vic_pos[6] = msg.transform.rotation.w

    def image_callback(self, msg):
        bridge = CvBridge()
        
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print("Error")
        else:
            # Save your OpenCV2 image as a jpeg
            if(self.count < self.size_of_dataset and (t.time() - self.t_start) >= self.capture_interval):  
                self.t_start = t.time()
                rospy.logwarn(f"Received the {self.count} image!") 
                self.count += 1
                cv2.imwrite(os.path.join(self.path, f'{self.count:08}.jpg'), cv2_img)
                
                # add pose to pose_list
                rospy.logwarn(f"{self.count} :   {self.vic_pos}")    
                self.all_poses.append({
                    'filename': f'{self.count:08}.jpg',
                    'pose': self.vic_pos.tolist()})

            elif self.count == self.size_of_dataset:
                with open(os.path.join(self.path, 'poses.json'), "w") as f:
                    json.dump(self.all_poses, f, indent=2)
    
                print(f"{self.count} image(s) of dataset was collected! \n")

def main(args=None):
    rospy.init_node('DatasetCollector', anonymous=True)

    sub = MinimalSubscriber()
    rospy.spin()   
    sub.destroy_node()
    rospy.shutdown()

if __name__ == '__main__':
    main()
