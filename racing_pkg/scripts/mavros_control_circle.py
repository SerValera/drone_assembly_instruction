#!/usr/bin/env python3
import logging
import rospy
import tf
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
import numpy as np
import math

pi_2 = math.pi / 2.0

class MavController:
    """
    A simple object to help interface with mavros
    """
    def __init__(self):
        self.id = 0
        self.name = '/red'
        logger = logging.getLogger(__name__)

        rospy.init_node('drone_control', anonymous=True)

        rospy.Subscriber(self.name + "/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber(self.name + "/mavros/rc/in", RCIn, self.rc_callback)

        self.cmd_pos_pub = rospy.Publisher(self.name + "/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher(self.name + "/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher(self.name + "/mavros/rc/override", OverrideRCIn, queue_size=1)
        self.attitude_pub = rospy.Publisher(self.name + '/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy(self.name + '/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy(self.name + '/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy(self.name + '/mavros/cmd/takeoff', CommandTOL)

        self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()
        self.current_position = self.get_current_position()

    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose

    def euler_from_quaternion(self, x, y, z, w):
                """
                Convert a quaternion into euler angles (roll, pitch, yaw)
                Args:
                    x (_type_): _description_
                    y (_type_): _description_
                    z (_type_): _description_
                    w (_type_): _description_

                Returns:
                    roll_x, pitch_y, yaw_z: is rotation around x, y, z in radians (counterclockwise)
                """            

                t0 = +2.0 * (w * x + y * z)
                t1 = +1.0 - 2.0 * (x * x + y * y)
                roll_x = math.atan2(t0, t1) 
                # roll_x = math.atan2(t0, t1) * (180 / math.pi)
                t2 = +2.0 * (w * y - z * x)
                t2 = +1.0 if t2 > +1.0 else t2
                t2 = -1.0 if t2 < -1.0 else t2
                pitch_y = math.asin(t2)
                # pitch_y = math.asin(t2) * (180 / math.pi)
                t3 = +2.0 * (w * z + x * y)
                t4 = +1.0 - 2.0 * (y * y + z * z)
                yaw_z = math.atan2(t3, t4)
                # yaw_z = math.atan2(t3, t4)* (180 / math.pi)
                return roll_x, pitch_y, yaw_z # in radians

    def get_current_position(self):
        self.current_position = self.pose.position
        current_orientation = self.pose.orientation
        current_xyz = np.array([self.current_position.x, self.current_position.y, self.current_position.z])
        curent_rpyw = np.array([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
        euler = self.euler_from_quaternion(curent_rpyw[0], curent_rpyw[1], curent_rpyw[2], curent_rpyw[3])
        rpy_deg = [round(euler[0] * 180 / math.pi, 1), round(euler[1] * 180 / math.pi, 1), round(euler[2] * 180 / math.pi, 1)]  
        return current_xyz, rpy_deg
   
    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        quat = tf.transformations.quaternion_from_euler(ro, pi, ya + pi_2)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.goto(pose)
        #print(quat)

    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=5.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        # Set to stabilize mode for arming
        #mode_resp = self.mode_service(custom_mode="0")
        mode_resp = self.mode_service(custom_mode="4")
        self.arm()

        # Set to guided mode
        #mode_resp = self.mode_service(custom_mode="4")

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        #return takeoff_resp
        return mode_resp

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()
    
    def get_distance3d(self, first_xyz, second_xyz):
        x1 = first_xyz[0]
        y1 = first_xyz[1]
        z1 = first_xyz[2]
        x2 = second_xyz[0]
        y2 = second_xyz[1]
        z2 = second_xyz[2]
        d = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
        return d



    # def navto(self, target_xyz, yaw=float('nan'), auto_arm=False, **kwargs):
    #     global delta
    #     current_xyz = self.get_current_position()
    #     delta = self.get_distance3d(current_xyz, target_xyz)
    #     x, y, z = target_xyz
    #     print('Going to: | x: {:.3f} y: {:.3f} z: {:.3f} yaw: {:.3f}'.format(x, y, z, yaw))


    def flip(self, min_z = 2.0, frame_id = 'map'):
        TOLERANCE = 0.2
        br = Vector3(30, 0, 0)
        current_xyz, rpy_deg = self.get_current_position()
        if current_xyz[2] < min_z - TOLERANCE:
            print("Can't do flip! Flip failed!")
            return False
        else:                    
            print("Flip started!")
            self.attitude_pub.publish(thrust=1)  # bump up
            rospy.sleep(0.2)

            self.attitude_pub.publish(body_rate=br, thrust=0.2) # maximum roll rate

            while True:
                current_xyz, rpy_deg = self.get_current_position()
                current_roll = rpy_deg[0]
                if abs(current_roll) > math.pi/2:
                    break

            print('Flip succeeded!')
            # self.navto(x=current_xyz[0], y=current_xyz[1], z=current_xyz[2], yaw=rpy_deg[2], frame_id=frame_id)   
            # finish flip

            return True
        
    def eight_trajectory(self, radius, altitude):
        radius = 2
        rate = rospy.Rate(15)
        for i in range(180):
            theta = i * 2.0 * math.pi / 180.0
            x = 3.0 + radius * math.cos(theta)
            y = 3.0 + radius * math.sin(theta) * math.cos(theta)
            z = altitude
            self.goto_xyz_rpy(x, y, z, 0.0, 0.0, theta)
            rate.sleep()
        rospy.sleep(3)

        self.goto_xyz_rpy(2.0, 6.0, altitude, 0, 0, 0)
        rospy.sleep(3)


    def circle_trajectory(self, radius, altitude):
        radius = 1
        rate = rospy.Rate(15)
        for i in range(180):
            theta = i * 2.0 * math.pi / 180.0
            x = 3.0 + radius * math.cos(theta)
            y = 3.0 + radius * math.sin(theta)
            z =  ((0.8 / (theta + 1)) + 0.5)    # altitude
            self.goto_xyz_rpy(x, y, z, 0.0, 0.0, -theta)
            rate.sleep()
        rospy.sleep(3)

        self.goto_xyz_rpy(2.0, 6.0, altitude, 0, 0, 0)
        rospy.sleep(3)


    def square_trajectory(self, altitude):
        print("Waypoint 0: position control")
        self.goto_xyz_rpy(0,0.0,altitude,0,0,0)
        rospy.sleep(3)
        print("Waypoint 1: position control")
        self.goto_xyz_rpy(5,0.0,altitude,0,0,0)
        rospy.sleep(5)
        print("Waypoint 2: position control")
        self.goto_xyz_rpy(5,5,altitude,0,0,0)
        rospy.sleep(3)
        print("Waypoint 3: position control")
        self.goto_xyz_rpy(0,5,altitude,0,0,0)
        rospy.sleep(3)
        print("Waypoint 4: position control")
        self.goto_xyz_rpy(0,0,altitude,0,0,0)
        rospy.sleep(3)
    
    def simple_demo(self):
        """
        A simple demonstration of using mavros commands to control a UAV.
        """
        altitude = 1
        radius = 1.0
        # self.goto_xyz_rpy(4.0, 4.0, altitude, 0, 0, 0)
        rospy.sleep(3)
        self.circle_trajectory(radius, altitude)
        # self.flip()


        # self.goto_xyz_rpy(0.0, 0.0, altitude, 0, 0, 0)
        # rospy.sleep(3)

if __name__=="__main__":
    commanding = MavController()
    rospy.sleep(1)
    commanding.simple_demo()