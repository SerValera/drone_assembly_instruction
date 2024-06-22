#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import tf
import tf2_ros
from scipy.spatial.transform import Rotation as R
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import *

RATE = 100


    

def callback(event):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    try:
        trans = tfBuffer.lookup_transform("camera_link", 'drone_frame', rospy.Time.now())
        poseSt = PoseStamped()
        poseSt.header.frame_id = "camera_link"
        poseSt.header.stamp = rospy.Time.now()

        poseSt.pose.position.x = trans.transform.translation.x
        poseSt.pose.position.y = trans.transform.translation.y
        poseSt.pose.position.z = trans.transform.translation.z

        poseSt.pose.orientation.x = trans.transform.rotation.x
        poseSt.pose.orientation.y = trans.transform.rotation.y
        poseSt.pose.orientation.z = trans.transform.rotation.z
        poseSt.pose.orientation.w = trans.transform.rotation.w


        pub.publish(poseSt)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)




if __name__ == '__main__':
    rospy.init_node('converter', anonymous=True)

    quat = quaternion_from_euler(0, 0, 0)


    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "camera_link"
    static_transformStamped.child_frame_id = "drone_frame"

    static_transformStamped.transform.translation.x = -0.075
    static_transformStamped.transform.translation.y = 0
    static_transformStamped.transform.translation.z = -0.032

    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)

    # timer = rospy.Timer(rospy.Duration(1/RATE), callback)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    pub = rospy.Publisher('/red/mavros/vision_pose/pose', PoseStamped, queue_size=1)

    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("camera_odom_frame", 'drone_frame', rospy.Time.now(), rospy.Duration(1))
            poseSt = PoseStamped()
            poseSt.header.frame_id = "world"
            poseSt.header.stamp = rospy.Time.now()

            poseSt.pose.position.x = trans.transform.translation.x
            poseSt.pose.position.y = trans.transform.translation.y
            poseSt.pose.position.z = trans.transform.translation.z

            poseSt.pose.orientation.x = trans.transform.rotation.x
            poseSt.pose.orientation.y = trans.transform.rotation.y
            poseSt.pose.orientation.z = trans.transform.rotation.z
            poseSt.pose.orientation.w = trans.transform.rotation.w


            pub.publish(poseSt)
            rate.sleep()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            rate.sleep()
            continue

