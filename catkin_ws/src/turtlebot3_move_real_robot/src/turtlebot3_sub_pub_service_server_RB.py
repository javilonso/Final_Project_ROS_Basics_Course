#! /usr/bin/env python 

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from turtlebot3_move.srv import MoveInSquare, MoveInSquareResponse
from lidar_range_dist import lidar_pos_front

PI = 3.1429
laser_distance_front = 1 
LIDAR_POS = lidar_pos_front


# =========================== FUNCTIONS ===========================
def laser_scan_callback(msg):
    # Reading front laser distance
    global laser_distance_front 
    laser_distance_front = msg.ranges[LIDAR_POS]

def service_callback(request):
    # Draw square with robot
    rospy.loginfo("[Square Service Server]: Robot performing Square")
    my_response = MoveInSquareResponse()

    # Square 4 sides
    for i in range (4):
        my_response.complete = do_square_straight_line()  #15 cm straight line

        # Obstacle found, robot stopped and error msg
        if my_response.complete == False:
            return my_response

        # Turn 90 degree robot
        do_square_turn() 
    
    return my_response

def do_square_straight_line():
    # Move robot 15cm straight line
    global laser_distance_front
    rospy.loginfo("[Square Service Server]: Robot going straight")

    move = Twist()
    move.linear.x = 0.02

    # while pub_cmd.get_num_connections() < 1:
    #     pass

    t0 = rospy.Time.now().to_sec()
    pub_cmd.publish(move)
    t1 = rospy.Time.now().to_sec()

    while abs(move.linear.x*(t1-t0)) < 0.15:
        t1 = rospy.Time.now().to_sec()
            # Stop robot in case obstacle at 0.2m
        if laser_distance_front < 0.2:
            rospy.loginfo("[Square Service Server]: Obstacle detected, stopping robot")
            stop_robot()
            return False
    
    return True

def do_square_turn():
    # Turn robot 90 degree
    stop_robot()
    rospy.loginfo("[Square Service Server]: Robot turning")

    #Converting from angles to radians
    speed=8
    angle=90
    angular_speed = speed*2*PI/360
    relative_angle = (angle+3.8)*2*PI/360

    move = Twist()
    move.angular.z = -abs(angular_speed)

    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    # while pub_cmd.get_num_connections() < 1:
    #     pass
    pub_cmd.publish(move)

    while(current_angle < relative_angle):
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)

    stop_robot()

def stop_robot():
    move = Twist()
    move.linear.x = 0.0
    move.angular.z = 0.0

    while pub_cmd.get_num_connections() < 1:
        pass
    pub_cmd.publish(move)

    rospy.loginfo("[Square Service Server]: Robot stopped")

# =========================== SERVICE SERVER MAIN =========================== 

# Initiate a node for service
rospy.init_node('turtlebot3_service_server') 

# Velocity Publisher
pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
# Laser Subscriber
sub_laser = rospy.Subscriber('/scan', LaserScan, laser_scan_callback)

# Service Server
my_service = rospy.Service('/move_in_square', MoveInSquare, service_callback)

rate = rospy.Rate(1)
rospy.spin()