#! /usr/bin/env python 

import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from turtlebot3_move.srv import MoveInSquare, MoveInSquareRequest
from turtlebot3_move.msg import OdomRecordAction, OdomRecordResult, OdomRecordFeedback
from lidar_range_dist import lidar_pos_front

ROBOT_MOVING = 0
LIDAR_POS = lidar_pos_front
print(lidar_pos_front)
# =========================== FUNCTIONS ===========================
def move_robot(): 
    global ROBOT_MOVING
    # Move robot in a straight line
    if ROBOT_MOVING == 0:
        move = Twist()
        move.linear.x = 0.02
        ROBOT_MOVING = 1

        # while pub_cmd.get_num_connections() < 1:
        #     pass
        pub_cmd.publish(move)
        rospy.loginfo("[MAIN] Robot moving")

def stop_robot():
    global ROBOT_MOVING
    # Stop robot
    if ROBOT_MOVING == 1 :
        move = Twist()
        move.linear.x = 0.0
        move.angular.z = 0.0
        ROBOT_MOVING = 0

        # while pub_cmd.get_num_connections() < 1:
        #     pass
        pub_cmd.publish(move)
        rospy.loginfo("[MAIN] Robot stopped")

def laser_scan_callback(msg):
    # Stop robot in case obstacle at 0.2m
    if msg.ranges[LIDAR_POS] < 0.2:
        rospy.loginfo("[MAIN] Obstacle detected, stopping robot")
        stop_robot()

def action_feedback_callback(feedback):
    # Feedback received from Action Server
    cur_distance = feedback

# =========================== MAIN =========================== 

# Initiate node
rospy.init_node('turtlebot3_move') 
rate = rospy.Rate(1)
rospy.loginfo("[MAIN] ========== (STARTING ROSJECT) ==========")


# Velocity Publisher
pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
# Laser Subscriber
sub_laser = rospy.Subscriber('/scan', LaserScan, laser_scan_callback)

# Call Action Server
client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)
client.wait_for_server()
goal = None
client.send_goal(goal, feedback_cb=action_feedback_callback)

# Call Do-Square Service Server
rospy.wait_for_service('/move_in_square')
execute_square_service_client = rospy.ServiceProxy('/move_in_square', MoveInSquare)
execute_square_request = MoveInSquareRequest()
service_result = execute_square_service_client(execute_square_request)
rospy.loginfo("[MAIN] ========== (SQUARE FINISHED) ==========")

rospy.loginfo("[MAIN] ========== (GOING STRAIGHT) ==========")
while client.get_state() < 2:
    # Moving straight looking for a wall
    move_robot()
    # rospy.spin()

rospy.loginfo("[MAIN] ========== Action Result ========== ")
print(client.get_result())