#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose, Point
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import StreamRate, CommandBool, SetMode, CommandTOL
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Path
import rospy
from scipy.spatial.distance import cdist

RATE = 25

class KeyboardController():

    def __init__(self):

        self.connected = False
        self.armed = False
        self.mode = "standby"
        self.path_ready = False
        self.pose_coord_local = np.zeros(3)
        self.obs_coord_local  = np.zeros(3)
        self.track = None
        self.current_track_point = None
        self.joint_traj = None

        self.name = "red"

        # Subscribers
        rospy.Subscriber("/%s/mavros/state" % self.name, State, self.callback_state, queue_size=10)
        rospy.Subscriber('/%s/mavros/vision_pose/pose' % self.name, PoseStamped, self.drone_pose_callback)
        rospy.Subscriber('/blue/mavros/vision_pose/pose' , PoseStamped, self.drone2_pose_callback)
        rospy.Subscriber('/director/scene', String, self.scene_callback)
        rospy.Subscriber("/director/track", Path, self.track_callback)
        rospy.Subscriber("/chomp_planner/planned_trajectory", MultiDOFJointTrajectory, self.path_planner_callback)

        # Publishers
        self.setpoint_target_pub = rospy.Publisher('/%s/mavros/setpoint_raw/local' % self.name, PositionTarget, queue_size=1)
        self.start_pub = rospy.Publisher('/chomp_planner/start_position', Pose, queue_size=1)
        self.goal_pub = rospy.Publisher('/chomp_planner/goal_position', Pose, queue_size=1)
        self.joint_path_pub = rospy.Publisher("/%s/mavros/setpoint_trajectory/local" % self.name, MultiDOFJointTrajectory, queue_size=1)
        self.obs_point_pub = rospy.Publisher('/chomp_planner/obstacles/pointcloud', PointCloud, queue_size=1)

        # Services
        service_timeout = 30
        rospy.logwarn("Waiting for ROS services")
        try:
            rospy.wait_for_service('/%s/mavros/set_mode' % self.name, service_timeout)
            rospy.wait_for_service('/%s/mavros/cmd/arming' % self.name, service_timeout)
            rospy.wait_for_service('/%s/mavros/cmd/takeoff' % self.name, service_timeout)
            rospy.wait_for_service('/%s/mavros/set_stream_rate' % self.name, service_timeout)
        except rospy.ROSException:
            rospy.logerr("failed to connect to services")

        self.set_mode_client = rospy.ServiceProxy('/%s/mavros/set_mode' % self.name, SetMode)
        self.arming_client = rospy.ServiceProxy('/%s/mavros/cmd/arming' % self.name, CommandBool)
        self.takeoff_client = rospy.ServiceProxy('/%s/mavros/cmd/takeoff' % self.name, CommandTOL)
        self.service_set_stream_rate = rospy.ServiceProxy('/%s/mavros/set_stream_rate' % self.name, StreamRate)

        try:
            rospy.logwarn("Setting stream rate to 100")
            resp = self.service_set_stream_rate(0, 100, True)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

        # Wait until connected to drone
        while not self.connected:
            rate.sleep()

        rospy.logwarn("Ready for flight")   

        self.timer = None 
        self.current_goal_pose = None


    def path_planner_callback(self, msg:MultiDOFJointTrajectory):
        self.joint_traj = msg

    def track_callback(self, msg:Path):
        self.track = []
        for point in msg.poses:
            self.track.append(np.array([point.pose.position.x, 
                                        point.pose.position.y, 
                                        point.pose.position.z]))
        
        self.track = np.array(self.track)
        self.timer = rospy.Timer(rospy.Duration(1. / RATE), self.routine)     


    def scene_callback(self, msg):
        scene = msg.data
        if scene == "red takeoff":
            self.takeoff()
            rospy.sleep(5)
            self.takeoff_pose = self.pose_coord_local
        if scene == "red exec follow":
            self.mode = "follow"
        if scene == "red overtake":
            self.mode = "overtake"
    
    def routine(self, event):
        pose = np.array([self.obs_coord_local])
        current_track_arg = np.argmin(cdist(pose, self.track)[0])
        current_goal_arg = current_track_arg + 30
        if current_goal_arg >= self.track.shape[0] - 1:
            current_goal_arg -= self.track.shape[0] - 1 
        pub_goal = Pose()
        pub_goal.position.x = self.track[current_goal_arg][0]
        pub_goal.position.y = self.track[current_goal_arg][1]
        pub_goal.position.z = self.track[current_goal_arg][2]

        self.goal_pub.publish(pub_goal)

        if self.mode == "follow":

        
            if self.current_track_point is not None:
                if np.linalg.norm(self.obs_coord_local - self.track[self.current_track_point]) >= 1:
                    target = PositionTarget()
                    target.coordinate_frame = 1
                    target.position.x = self.track[self.current_track_point][0]
                    target.position.y = self.track[self.current_track_point][1]
                    target.position.z = self.track[self.current_track_point][2]
                    target.yaw = 0
                    
                    self.setpoint_target_pub.publish(target)
                    if np.linalg.norm(self.pose_coord_local - self.track[self.current_track_point]) <= 0.5:
                        self.current_track_point += 1
                    if self.current_track_point >= self.track.shape[0]:
                        self.current_track_point = 0
            else:
                if np.linalg.norm(self.obs_coord_local - self.track[0]) >= 1:
                    self.current_track_point = 0
                
        
        elif self.mode == "overtake":
            if self.joint_traj is not None:
                self.joint_path_pub.publish(self.joint_traj)
                self.current_goal_pose = np.array([self.joint_traj.points[-1].transforms[0].translation.x, 
                                                   self.joint_traj.points[-1].transforms[0].translation.y, 
                                                   self.joint_traj.points[-1].transforms[0].translation.z])

                self.mode = 'standby'
            self.current_track_point = current_goal_arg
        if self.current_goal_pose is not None:
            if np.linalg.norm(self.pose_coord_local - self.current_goal_pose) <= 1:
                self.mode = 'follow'

    
    def set_vel(self, vel):
        vel_cmd = TwistStamped()
        vel_cmd.header.frame_id = "map"
        vel_cmd.header.stamp = rospy.Time.now()
        vel_cmd.twist.linear.x = vel[0]
        vel_cmd.twist.linear.y= vel[1]
        vel_cmd.twist.linear.z = vel[2]
        vel_cmd.twist.angular.z = vel[3]
        self.vel_publisher.publish(vel_cmd)

    def callback_state(self, state):
        """
        Subscriber callback function from the "mavros/state" topic.
        Is used to save the connection and armed states of the drone.

        param state: a ros message that contains the state of the drone
        """
        self.connected = state.connected
        self.armed = state.armed


    def drone_pose_callback(self, msg):
        """
        The callback function for the mavros/local_position/pose topic. Updates the current drone
        position and the is_grounded status.

        param msg: A ROS message of type PoseStamped.
        """
        # rospy.logwarn(msg)
        self.pose_local = msg
        self.pose_coord_local = np.array([self.pose_local.pose.position.x,
                                          self.pose_local.pose.position.y, self.pose_local.pose.position.z])
        pub_start = Pose()
        pub_start.position.x = self.pose_coord_local[0]
        pub_start.position.y = self.pose_coord_local[1]
        pub_start.position.z = self.pose_coord_local[2]

        self.start_pub.publish(pub_start)
        
    def drone2_pose_callback(self, msg):
        """
        The callback function for the mavros/local_position/pose topic. Updates the current drone
        position and the is_grounded status.

        param msg: A ROS message of type PoseStamped.
        """
        # rospy.logwarn(msg)
        self.pose_local = msg
        self.obs_coord_local = np.array([self.pose_local.pose.position.x,
                                          self.pose_local.pose.position.y, self.pose_local.pose.position.z])
        
        obs_pointcloud = PointCloud()
        obs_pointcloud.header.frame_id = 'map'
        obs_pointcloud.header.stamp = rospy.Time.now()
        point = Point()
        point.x = self.obs_coord_local[0]
        point.y = self.obs_coord_local[1]
        point.z = self.obs_coord_local[2]
        obs_pointcloud.points.append(point)
        self.obs_point_pub.publish(obs_pointcloud)
        
    def arm(self):
        """
        Sets the drone mode to "GUIDED" and arms the drone, enabling it to complete missions and tasks.
        """

        # Changes mode to "GUIDED"
        try:
            rospy.logwarn("sending mode")
            resp = self.set_mode_client(0, 'GUIDED')
            while not resp.mode_sent:
                resp = self.set_mode_client(0, 'GUIDED')
                rate.sleep()
            if resp.mode_sent:
                rospy.logwarn("Mode changed")
                # self.mode = "GUIDED"
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

        # Arms the drone
        try:
            rospy.logwarn("sending arm request")
            resp = self.arming_client(True)
            while not resp.success:
                resp = self.arming_client(True)
                rate.sleep()
            if resp.success:
                rospy.logwarn("armed")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)


    def takeoff(self):
        """
        The takeoff strategy for the drone.
        """

        # Arm the drone
        if not self.armed:
            self.arm()
        # Takeoff
        try:
            rospy.logwarn("sending takeoff request")
            resp = self.takeoff_client(0, 0, 0, 0, 0.5)
            while not resp.success:
                resp = self.takeoff_client(0, 0, 0, 0, 0.5)
                rate.sleep()
            if resp.success:
                rospy.logwarn("takeoff successful")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
        rospy.logwarn("takeoff END")



    def land(self):
        """
        Landing strategy for the drone.
        Lands the drone by simply switching the mode to "LAND"
        """

        try:
            rospy.logwarn("Sending mode")
            resp = self.set_mode_client(0, 'LAND')
            while not resp.mode_sent:
                resp = self.set_mode_client(0, 'LAND')
                rate.sleep()
            if resp.mode_sent:
                rospy.logwarn("Mode changed to landing")
                # self.mode = "LAND"
                self.is_grounded = True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

def shutdown():
    """
    Function is called on shutdown.
    """

    # Land the drone
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    try:
        rospy.logwarn("Sending mode")
        resp = set_mode_client(0, 'LAND')
        while not resp.mode_sent:
            resp = set_mode_client(0, 'LAND')
            rate.sleep()
        if resp.mode_sent:
            rospy.logwarn("Mode changed to landing")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
    rospy.logwarn("Ground station node shutting down")

if __name__ == "__main__":
    try:
        rospy.on_shutdown(shutdown)
        rospy.init_node('controller', anonymous=True)
        rate = rospy.Rate(100)
        controller = KeyboardController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass