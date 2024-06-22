#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Transform, Twist
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Path
import bezier
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


def get_setpoint(x, y, z, yaw=0.0, in_rads=False): # early yaw=np.pi/2
    """ Set coordinates to PoseStamped() object

    Args:
        x (float): x coordinate
        y (float): y coordinate
        z (float): z coordinate
        yaw (float, optional): drone orientation. Defaults to 0.

    Returns:
        set_pose (PoseStamped): _description_
    """    
    set_pose = PoseStamped()
    set_pose.pose.position.x = x
    set_pose.pose.position.y = y
    set_pose.pose.position.z = z
    if not in_rads:
        q = quaternion_from_euler(0, 0, yaw * 2 * np.pi / 360.0)
    else:
        q = quaternion_from_euler(0, 0, yaw)
    set_pose.pose.orientation.x = q[0]
    set_pose.pose.orientation.y = q[1]
    set_pose.pose.orientation.z = q[2]
    set_pose.pose.orientation.w = q[3]
    return set_pose 

def create_scan_path(max_point, min_point, height, steps):

    side_len = np.abs(max_point[0] - min_point[0])/steps
    side_width = np.abs(max_point[1] - min_point[1])
    path = []

    current_point = np.array([max_point[0], max_point[1], height, np.pi/2])

    for i in range(steps):
        if (i % 2) == 0:  
            path.append(current_point.copy())
            current_point = current_point + np.array([-side_len, 0, 0, 0])
            path.append(current_point.copy())
            current_point = current_point + np.array([0, -side_width/2, 0, 0])
            current_point[3] = 0
            path.append(current_point.copy())
            current_point[3] = np.pi
            path.append(current_point.copy())
            current_point = current_point + np.array([0, -side_width/2, 0, 0])
            current_point[3] = 3*np.pi/2
        else:  
            path.append(current_point.copy())
            current_point = current_point + np.array([-side_len, 0, 0, 0])
            path.append(current_point.copy())
            current_point = current_point + np.array([0, side_width/2, 0, 0])
            current_point[3] = 0
            path.append(current_point.copy())
            current_point[3] = np.pi
            path.append(current_point.copy())
            current_point = current_point + np.array([0, side_width/2, 0, 0])
            current_point[3] = np.pi/2
        path.append(current_point.copy())
    return np.array(path)

def circle_trajectory(radius=0.05, altitude=1):
    path = []
    for i in range(6*180):
        theta = i * 2.0 * np.pi / 180.0
        x = radius * np.cos(theta) + -2
        y = radius * np.sin(theta)
        z =  (0.4 + 1.5 * i * 1 / (6*180)) #((0.8 / (theta + 1)) + 0.5)    # altitude
        yaw = (theta + np.pi) - np.pi / 6 - 0.174533
        path.append(np.array([x, y, z, yaw]))

    return np.array(path)

def rect_traj():
    path = np.array([[-3.6, -1.6, 1.5, 0],
                    [-3.6, -1.6, 1.5, np.pi/2],
                     [2.5, -0.9, 1.5, np.pi/2],
                     [2.5, -0.9, 1.5, np.pi],
                     [2.5, -0.9, 2.5, np.pi], # point
                     [2.1, 0.3, 2.5, np.pi],
                     [1.9, 1.2, 2.5, np.pi],
                     [0.9, 1.3, 2.5, np.pi],
                     [0.9, 1.3, 2.5, 3*np.pi/2],
                     [0, 1.0, 1.5, 3*np.pi/2],
                     [-3.9, 1.3, 1.5, 3*np.pi/2]])

    return path

def scan_traj():
    path = np.array([[-0.5, 1, 1.4, 0],
                     [0.2, -0.8, 1.4, np.pi/4],
                     [2.2, -1.5, 1.4, np.pi/2]])

    return np.array(path)


def R(alpha, beta, gamma):

    Rx = np.array([[ 1, 0           , 0           ],
                    [ 0, np.cos(alpha),-np.sin(alpha)],
                    [ 0, np.sin(alpha), np.cos(alpha)]])

    Ry = np.array([[ np.cos(beta), 0, np.sin(beta)],
                    [ 0           , 1, 0           ],
                    [-np.sin(beta), 0, np.cos(beta)]])

    Rz = np.array([[ np.cos(gamma), -np.sin(gamma), 0 ],
                    [ np.sin(gamma), np.cos(gamma) , 0 ],
                    [ 0           , 0            , 1 ]])

    return Rx @ Ry @ Rz

def get_race_track(points, safe_dist=1.5, num_of_points=20):
        s_vals = np.linspace(0.0, 1.0, num_of_points)
        one = np.array([1, 0, 0])
        track = []

        key_array = []

        for i in range(points.shape[0] - 1):
            keys = np.array([points[i, :3], 
                        points[i, :3] + safe_dist * R(points[i, 3], points[i, 4], points[i, 5]) @ one,  
                        points[i + 1, :3] - safe_dist * R(points[i + 1, 3], points[i + 1, 4], points[i + 1, 5]) @ one,
                        points[i + 1, :3]])
            key_array.append(keys)
            curve = bezier.Curve([keys[:, 0], keys[:, 1], keys[:, 2]], degree=len(keys)-1)
            np_curve = curve.evaluate_multi(s_vals[:-1])
            output_paths = np_curve.reshape(3, num_of_points-1).transpose(1, 0)
            track.append(output_paths)
            
        
        keys = np.array([points[-1, :3], 
                        points[-1, :3] + safe_dist * R(points[-1, 3], points[-1, 4], points[-1, 5]) @ one,  
                        points[0, :3] - safe_dist * R(points[0, 3], points[0, 4], points[0, 5]) @ one,
                        points[0, :3]])
        key_array.append(keys)

        curve = bezier.Curve([keys[:, 0], keys[:, 1], keys[:, 2]], degree=len(keys)-1)
        np_curve = curve.evaluate_multi(s_vals[:-1])
        output_paths = np_curve.reshape(3, num_of_points-1).transpose(1, 0)
        track.append(output_paths)
        track = np.vstack(track)
        print(track)
        return track

def publish_path(traj):
    path_pub = rospy.Publisher("/gs/track", Path, queue_size=1)
    path = Path()
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = '/vicon/world'
    for point in traj:
        pose = PoseStamped()
        pose.header.frame_id = '/vicon/world'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        path.poses.append(pose)
    path_pub.publish(path)

def talker():
    # gate1_pose = np.array([-4, -1.6,  1.6,  0, 0,  0])
    # gate2_pose = np.array([-0.2,  -2.1,  1.6,  0, 0,  0])
    # gate3_pose = np.array([3.5,  -1.8,  1.6,  0, 0,  np.pi/2])
    # gate4_pose = np.array([3.5, 1.7,  1.6,  0, 0,  np.pi])
    # gate5_pose = np.array([-0.2, 2.4,  1.6,  0, 0,  np.pi])
    # gate6_pose = np.array([-3.4, 1.9,  1.6,  0, 0,  -np.pi/2])
    scale = 0.5
    gate1_pose = np.array([-4 * scale, -1.6* scale,  1.4,  0, 0,  0]) 
    gate2_pose = np.array([-0.2* scale,  -2.1* scale,  1.4,  0, 0,  0]) 
    gate3_pose = np.array([3.7* scale,  -1.8* scale,  1.4,  0, 0,  np.pi/2]) 
    gate4_pose = np.array([3.7* scale, 1.7* scale,  1.4,  0, 0,  np.pi]) 
    gate5_pose = np.array([-0.2* scale, 2.4* scale,  1.4,  0, 0,  np.pi]) 
    gate6_pose = np.array([-3.4* scale, 1.9* scale,  1.4,  0, 0,  -np.pi/2]) 

    points = np.array([gate1_pose, gate2_pose, gate3_pose, gate4_pose, gate5_pose, gate6_pose])
    scan_path = get_race_track(points, 0)
    
    # scan_path = create_scan_path((0, 1), (-3.2, -1), 1, 3)
    # scan_path = circle_trajectory(1)
    # scan_path = rect_traj()
    # scan_path = scan_traj()
    current_point = 0
    pub = rospy.Publisher('/red/mavros/setpoint_position/local', PoseStamped, queue_size=1)
    setpoint_path_pub = rospy.Publisher("/red/mavros/setpoint_trajectory/local", MultiDOFJointTrajectory, queue_size=1)
    rospy.init_node('Ground_Station', anonymous=True)
    rate = rospy.Rate(100)
    command = 0
    prev_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if command == 0 or command == 1:
            command = int(input("Command 1 to go to pose, 2 to scan "))
        
        if command == 1:
            x = float(input("input x "))
            y = float(input("input y "))
            z = float(input("input z "))
            yaw = float(input("input yaw "))
            setpoint = get_setpoint(x, y, z, yaw)
            setpoint.header.stamp = rospy.Time.now()
            pub.publish(setpoint)
            rospy.logwarn(f"publishing : {setpoint}" )
        if command == 2:
            publish_path(scan_path)

            ss = np.linspace(0, 1, scan_path.shape[0])
            path = ta.SplineInterpolator(ss, scan_path)

            vlim = np.array([[-0.25, 0.25], [-0.25, 0.25], [-0.25, 0.25]])
            alim = np.array([[-0.75, 0.75], [-0.75, 0.75], [-0.75, 0.75]])
            pc_vel = constraint.JointVelocityConstraint(vlim)
            pc_acc = constraint.JointAccelerationConstraint(alim)

            instance = algo.TOPPRA([pc_vel, pc_acc], path)
            jnt_traj = instance.compute_trajectory(0, 0)

            t = np.linspace(0, jnt_traj.duration, 100)
            
            qs = jnt_traj(t)
            qds = jnt_traj(t, 1)
            qdds = jnt_traj(t, 2)

            qdds[-1, 0] = 0
            qdds[-1, 1] = 0
            qdds[-1, 2] = 0


            traj = MultiDOFJointTrajectory()
            traj.header.stamp = rospy.Time.now()
            traj.header.frame_id = 'map'
            current_time = 0
            length = 0
            for i in range(len(qs)):
                pose = MultiDOFJointTrajectoryPoint()
                transform = Transform()
                transform.translation.x = qs[i, 0]
                transform.translation.y = qs[i, 1]
                transform.translation.z = qs[i, 2]
                if i+1 < len(qs):
                    length += np.linalg.norm(qs[i+1] - qs[i])
                else:
                    print(length)
                quaternion = quaternion_from_euler(0, 0, np.arctan2(qs[i, 1], qs[i, 0]) - np.pi)
                transform.rotation.x = quaternion[0]
                transform.rotation.y = quaternion[1]
                transform.rotation.z = quaternion[2]
                transform.rotation.w = quaternion[3]
                pose.transforms.append(transform)

                # vels = Twist()
                # vels.linear.x = qds[i, 0]
                # vels.linear.y = qds[i, 1]
                # vels.linear.z = qds[i, 2]
                # pose.velocities.append(vels)

                # print(vels)
                # print("!!!!!!!!!!!!!!!!!!!")

                # acc = Twist()
                # acc.linear.x = qdds[i, 0]
                # acc.linear.y = qdds[i, 1]
                # acc.linear.z = qdds[i, 2]
                # pose.accelerations.append(acc)

                # print(acc)
                # print("!!!!!!!!!!!!!!!!!!!")



                traj.points.append(pose)
                pose.time_from_start = rospy.Duration(t[i] - current_time)
                current_time = t[i]

            setpoint_path_pub.publish(traj)
            command = 0

            # if rospy.Time.now().to_sec() - prev_time >= 7: # sweeping 0.01
            #     setpoint = get_setpoint(scan_path[current_point, 0], scan_path[current_point, 1], scan_path[current_point, 2], scan_path[current_point, 3], in_rads=True)
            #     setpoint.header.stamp = rospy.Time.now()
            #     pub.publish(setpoint)
            #     rospy.logwarn(f"publishing : {setpoint}")
            #     prev_time = rospy.Time.now().to_sec()
            #     current_point += 1
            #     if current_point >= scan_path.shape[0]:
            #         command = 0
            #         current_point = 0
            #     if current_point == 1:
            #         rospy.sleep(5)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass