#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

from geometry_msgs.msg import TransformStamped, PoseStamped



def callback(data):
    poseSt = PoseStamped()

    poseSt.header = data.header 

    poseSt.pose.position.x = data.transform.translation.x
    poseSt.pose.position.y = data.transform.translation.y
    poseSt.pose.position.z = data.transform.translation.z

    poseSt.pose.orientation = data.transform.rotation

    pub.publish(poseSt)


if __name__ == '__main__':
    rospy.init_node('converter', anonymous=True)

    rospy.Subscriber("/vicon/red/red", TransformStamped, callback)
    pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()