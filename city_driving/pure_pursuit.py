#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Point

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        #self.odom_topic       = rospy.get_param("~odom_topic", "/odom")
        self.odom_topic       = "/odom"
        self.lookahead        = rospy.get_param("~lookahead", 0.6)
        self.speed            = rospy.get_param("~speed", 1.0)
        self.wheelbase_length = 0.375
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.update_pursuit, queue_size=1)

        self.look_index = 0 # Ensures goal points are moving forward
        self.last_waypoint_index = 0 # Bookkeeps most recent waypoint passed
        self.last_goal = np.array([0,0]) 
        self.valid_trajectory = False  # Check to see if trajectory has been set.

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        # Reset trajectory tracking variables
        self.look_index = 0
        self.last_waypoint_index = 0
        self.last_goal = self.trajectory.points[0]
        self.valid_trajectory = True

    def update_pursuit(self, pose_msg):
        """
        Updates the current goal position, steering angle, and velocity of racecar when odometry
        of robot is updated.
        """

        # If trajectory isn't set, don't update drive messages. 
        if not self.valid_trajectory:
            return 

        position = pose_msg.pose.pose.position
        goal_map = self.look_ahead(position.x, position.y) # Instantaneous goal - lookahead point

        # Transform from world coord to robot.
        quat = pose_msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        sint = np.sin(euler[2])
        cost = np.cos(euler[2])
        robot_transform = np.array([[cost, -sint, 0, position.x], [sint, cost, 0, position.y], [0,0,1,0], [0,0,0,1]])
        goal_transform = np.array([[1, 0, 0, goal_map[0]], [0, 1, 0, goal_map[1]], [0,0,1,0], [0,0,0,1]])

        goal_car = np.linalg.inv(np.matmul(np.linalg.inv(goal_transform), robot_transform))
        goal = np.array([goal_car[0, -1], goal_car[1,-1]])
        
        # Calculate the proper steering angle.
        R = (self.lookahead*self.lookahead)/(2*goal[1])# Radius of curvature connecting these points
        eta = np.arctan(self.wheelbase_length / R)

        # Create the drive message.
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = eta
    
        # Check to see if we are close enough to the final destination to stop.
        pos = np.array([position.x, position.y])
        if np.linalg.norm(pos - self.last_goal) < 0.25:
            drive_msg.drive.speed = 0
        else: 
            drive_msg.drive.speed = self.speed

        self.drive_pub.publish(drive_msg)

        



    def look_ahead(self, x, y):
        """
        Returns the point on path that is instantaneously one look ahead distance away from rover
        If multiple intersections, selects "farthest along" point
        https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
        """
        # Ensures the last waypoint isn't picked up at the start. 
        if self.last_waypoint_index == 0:
            end_index = 2
        else:
            end_index = 0

        for i in range(self.last_waypoint_index, len(self.trajectory.points) - 1 - end_index):
            # Geometry of segment vector and vector from robot to start point
            point_1 = np.array(self.trajectory.points[i+1])
            point_2 = np.array(self.trajectory.points[i])
            seg_vec = point_1 - point_2
            rover_vec = point_2 - np.array([x, y])
            a = np.dot(seg_vec, seg_vec)
            b = 2 * np.dot(rover_vec, seg_vec)
            c = np.dot(rover_vec, rover_vec) - self.lookahead*self.lookahead
            discriminant = b*b-4*a*c

            if discriminant >= 0:
                # p1 and p2 are integer multiples of the segment vector corresponding to
                # where the intersection lies
                p1 = (-b + np.sqrt(discriminant))/(2*a)
                p2 = (-b - np.sqrt(discriminant))/(2*a)

                # p1 will always be farther than p2
                if p1 >= 0 and p1 <= 1:
                    # Successful intersection
                    if i+p1 > self.look_index: # Is this point moving forward?
                        self.look_index = i + p1
                        self.last_waypoint_index = i
                        self.last_goal = p1 * seg_vec + point_2
                        return p1 * seg_vec + point_2

                if p2 >= 0 and p2 <= 1:
                    # Successful intersection
                    if i+p2 > self.look_index: # Is this point moving forward?
                        self.look_index = i + p2
                        self.last_waypoint_index = i
                        self.last_goal = p2 * seg_vec + point_2
                        return p2 * seg_vec + point_2

        # If nothing found
        return self.last_goal


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
