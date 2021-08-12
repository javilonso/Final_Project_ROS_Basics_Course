#! /usr/bin/env python 

import rospy
import actionlib
from turtlebot3_move.msg import OdomRecordAction, OdomRecordResult, OdomRecordFeedback
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import time
from math import sqrt, pow
from datetime import datetime
from lidar_range_dist import lidar_pos_front

points = []
dist_total = 0
cur_dist = 0
old_x = None
old_y = None
cur_callback_num = 0
front_laser_dist = 1
t0 = None
t1 = None
LIDAR_POS = lidar_pos_front

class Robot_Action_Odom(object):

    # create messages that are used to publish feedback/result
    _feedback = OdomRecordFeedback()
    _result   = OdomRecordResult()

# =========================== FUNCTIONS ===========================
    
    # Initiate the Action Server
    def __init__(self):
        
        print("Done")
        self._as = actionlib.SimpleActionServer("record_odom", OdomRecordAction, self.goal_callback, False)
        self._as.start()

        # Odometry Subscriber and Laser Subscriber
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.laser_scan_callback, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)

        # publish info to the console for the user
        rospy.loginfo("[Odom. Action Server]: Waiting for order")

    # Action Server Callback 
    def goal_callback(self, goal):
        
        global points
        global dist_total
        global front_laser_dist

        rospy.loginfo("[Odom. Action Server]: Goal received")
        
        while True:
            # Builds the next feedback msg
            self._feedback.current_total = dist_total
            # publish the feedback
            self._as.publish_feedback(self._feedback)
            
            # Obstacle found, return Result and Finish
            if front_laser_dist < 0.2:
                aux_point = Point(points[0], points[1], points[2])
                self ._result.list_of_odoms = aux_point
                self._as.set_succeeded(self._result)
                break

    # Odometry Subscriber Callback
    def odom_callback(self, msg):
        global points
        global dist_total
        global cur_dist
        global old_x
        global old_y
        global cur_callback_num
        global t0, t1

        if cur_callback_num > 0:
            t1 = datetime.now()
            
            # Save new ODOMETRY ANGLES every second (result)
            if ((t1-t0).total_seconds() > 1.0):
                points = [msg.pose.pose.orientation.x, 
                            msg.pose.pose.orientation.y, 
                            msg.pose.pose.orientation.w]
                t0 = datetime.now()

            # Calculate TOTAL DISTANCE TRAVELLED (feedback)
            new_x = msg.pose.pose.position.x
            new_y = msg.pose.pose.position.y
            dx = new_x - old_x
            dy = new_y - old_y
            old_x = new_x
            old_y = new_y
            cur_dist = sqrt(pow(dx,2) + pow(dy,2))
            dist_total = dist_total + cur_dist

        # Record 1st Message from Odometry Subscriber
        else:
            points = [msg.pose.pose.orientation.x, 
                msg.pose.pose.orientation.y, 
                msg.pose.pose.orientation.w]
            t0 = datetime.now()

            old_x = msg.pose.pose.position.x
            old_y = msg.pose.pose.position.y

        cur_callback_num = cur_callback_num + 1

    def laser_scan_callback(self, msg):
        # Save front laser front distance in order to return action server result when >0.2m
        global front_laser_dist
        front_laser_dist = msg.ranges[LIDAR_POS]

# =========================== ACTION SERVER MAIN =========================== 

if __name__ == '__main__':
  rospy.init_node('robot_action_server')
  rospy.Rate(10)
  Robot_Action_Odom()
  rospy.spin()