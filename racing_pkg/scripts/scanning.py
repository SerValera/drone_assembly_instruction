#!/usr/bin/env python3
import sys
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
import rospy
from tf.transformations import euler_from_quaternion
import bezier

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


class DirectorNode():

    def __init__(self):
        
        self.scene_pub = rospy.Publisher('/director/scene', String, queue_size=1)
        self.path_pub = rospy.Publisher("/director/track", Path, queue_size=1)

        # rospy.Subscriber('/vicon/gate1/gate1', TransformStamped, self.gate1_callback)
        # rospy.Subscriber('/vicon/gate2/gate2', TransformStamped, self.gate2_callback)
        # rospy.Subscriber('/vicon/gate3/gate3', TransformStamped, self.gate3_callback)
        # rospy.Subscriber('/vicon/gate4/gate4', TransformStamped, self.gate4_callback)
        self.gate1_pose = np.array([-2.69561629e+00, -1.53825162e+00,  9.47194599e-01,  6.28273439e+00, 6.28297389e+00,  7.85285173e+00])
        self.gate2_pose = np.array([-1.87481121e+00,  9.02861319e-01,  7.56298315e-01,  1.12170670e-03, 6.27636366e+00,  6.28312348e+00])
        self.gate3_pose = np.array([1.27049242e+00,  1.06784123e+00,  1.47972163e+00,  6.28245563e+00, 6.27803346e+00,  1.19276758e-02])
        self.gate4_pose = np.array([2.25611317e+00, -1.16825784e+00,  8.03770481e-01,  6.27834101e+00, 6.28314144e+00,  4.68773432e+00])

        


        rospy.sleep(3)

        if self.gate1_pose is None or self.gate2_pose is None or self.gate3_pose is None or self.gate4_pose is None:
            rospy.logerr("Gates not found")
            self.track = self.circle_track(2)
        else:
            print(np.array([self.gate1_pose,
                                        self.gate2_pose,
                                        self.gate3_pose,
                                        self.gate4_pose]))
            self.track = self.get_race_track(np.array([self.gate1_pose,
                                        self.gate2_pose,
                                        self.gate3_pose,
                                        self.gate4_pose]))
        print()
        self.publish_path(self.track)

        rospy.logwarn("The curtain rises")        
        
        # Collect events until released
        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()


    def gate1_callback(self, msg):
        q = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z,
             msg.transform.rotation.w]
        angle = euler_from_quaternion(q)
        self.gate1_pose = np.array(
            [msg.transform.translation.x, 
             msg.transform.translation.y, 
             msg.transform.translation.z, 
             angle[0] % (2 * np.pi), 
             angle[1] % (2 * np.pi), 
             angle[2] % (2 * np.pi) + np.pi/2])
        

    def gate2_callback(self, msg):
        q = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z,
             msg.transform.rotation.w]
        angle = euler_from_quaternion(q)
        self.gate2_pose = np.array(
            [msg.transform.translation.x, 
             msg.transform.translation.y, 
             msg.transform.translation.z, 
             angle[0] % (2 * np.pi), 
             angle[1] % (2 * np.pi), 
             angle[2] % (2 * np.pi)])


    def gate3_callback(self, msg):
        q = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z,
             msg.transform.rotation.w]
        angle = euler_from_quaternion(q)
        self.gate3_pose = np.array(
            [msg.transform.translation.x, 
             msg.transform.translation.y, 
             msg.transform.translation.z, 
             angle[0] % (2 * np.pi), 
             angle[1] % (2 * np.pi), 
             angle[2] % (2 * np.pi)])


    def gate4_callback(self, msg):
        q = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z,
             msg.transform.rotation.w]
        angle = euler_from_quaternion(q)
        self.gate4_pose = np.array(
            [msg.transform.translation.x, 
             msg.transform.translation.y, 
             msg.transform.translation.z, 
             angle[0] % (2 * np.pi), 
             angle[1] % (2 * np.pi), 
             angle[2] % (2 * np.pi) -  np.pi/2])
    


    def get_race_track(self, points, safe_dist=1.5, num_of_points=20):
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
        # track = np.concatenate((track, track, track, track, track))
        return track


    def circle_track(self, radius):

        angles = np.linspace(0, 2 * np.pi, 100)
        x = radius * np.cos(angles)
        y = radius * np.sin(angles)
        z = 1 * np.ones(angles.shape[0])
        path = np.stack((x, y, z), axis=-1)

        return path
    
    def publish_path(self, traj):
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
        self.path_pub.publish(path)

    def on_release(self, key):
        pass


    def on_press(self, key):
        try:
            print('alphanumeric key {0} pressed'.format(
                key.char))
            

            if key.char == '1':
                self.scene_pub.publish('blue takeoff')
            if key.char == '2':
                self.scene_pub.publish('red takeoff')
            if key.char == '3':
                self.scene_pub.publish('blue fly trajectory')
            if key.char == '4':
                self.scene_pub.publish('red exec follow')
            if key.char == '5':
                self.scene_pub.publish('red overtake')
                

        except AttributeError:
            print('special key {0} pressed'.format(
                key))
            if key == Key.f10:
                sys.exit()


    

def shutdown():
    """
    Function is called on shutdown.
    """
    pass

if __name__ == "__main__":
    try:
        rospy.on_shutdown(shutdown)
        rospy.init_node('DirectorNode', anonymous=True)
        rate = rospy.Rate(10)
        controller = DirectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass