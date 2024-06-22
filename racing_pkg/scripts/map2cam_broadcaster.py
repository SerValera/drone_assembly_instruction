#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import TransformStamped

def camera_pose_callback(msg):
    br = tf.TransformBroadcaster()
    pose = msg.transform
    br.sendTransform((pose.translation.x, pose.translation.y, pose.translation.z),
                     (pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
                     rospy.Time.now(),
                     "camera_depth_frame",
                     "world")

if __name__ == '__main__':
    rospy.init_node('map2cam_tf')
    rospy.loginfo("Broadcasting")
    rospy.Subscriber('/vicon/red/red',
                     TransformStamped,
                     camera_pose_callback)
    rospy.spin()