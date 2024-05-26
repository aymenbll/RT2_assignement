#!/usr/bin/env python

"""
.. module:: Display_speed
   :platform: Unix
   :synopsis: This ROS node manages robot goal setting and status monitoring by receiving user inputs, and handling feedback and odometry data.

.. moduleauthor:: Aimen.B s5922996@studenti.unige.it
   This node implements a controller for the Buggy.

This node subscribes to the following topics:
    - /odom (nav_msgs/Odometry)
        Receives the robot's instantaneous position and velocity.
    - /reaching_goal/goal (assignment_2_2023/PlanningActionGoal)
        Receives the last goal target chosen by the user.

And provides the following service:
    - /speed_distance (assignment_2_2023/Avrg)
        Provides the average speed and distance of the robot from the goal.

"""

import rospy
from nav_msgs.msg import Odometry
from assignment_2_2023.msg import PlanningActionGoal
from assignment_2_2023.srv import Avrg, AvrgResponse
import math

# Global variables to store robot's position, velocity, target coordinates, and calculated distance and speed
dis_x = 0.0
dis_y = 0.0
speed_x = 0.0
speed_y = 0.0
final_x = 0.0
final_y = 0.0
dx = 0.0
dy = 0.0
d = 0.0
spd = 0.0
spdf = 0.0

def odom_callback(msg):
    """
    Callback function to get the instantaneous position coordinates and velocity from the odom topic.

    Args:
        msg (nav_msgs.msg.Odometry): Odometry message containing robot's instantaneous position and velocity.
    """
    global dis_x, dis_y, speed_x, speed_y
    dis_x = msg.pose.pose.position.x
    dis_y = msg.pose.pose.position.y
    speed_x = msg.twist.twist.linear.x
    speed_y = msg.twist.twist.linear.y

def goal_callback(msg):
    """
    Callback function to get the last goal target chosen by the user.

    Args:
        msg (assignment_2_2023.msg.PlanningActionGoal): Goal message containing the last goal target.
    """
    global final_x, final_y
    final_x = msg.goal.target_pose.pose.position.x
    final_y = msg.goal.target_pose.pose.position.y

def speed_distance_callback(req):
    """
    Service callback function to provide the average speed and distance from the target.

    Args:
        req (assignment_2_2023.srv.AvrgRequest): Service request.

    Returns:
        assignment_2_2023.srv.AvrgResponse: Service response containing distance and average speed.
    """
    global d, spdf

    spd_sum = 0.0
    for _ in range(50):
        spd_sum += math.sqrt(speed_x**2 + speed_y**2)
        rospy.sleep(0.1)
    spdf = spd_sum / 50.0

    dx = final_x - dis_x
    dy = final_y - dis_y
    d = math.sqrt(dx**2 + dy**2)

    return AvrgResponse(d, spdf)

def main():
    """
    Main function to initialize the ROS node, subscribers, and service server.
    The main function performs the following steps:
    
    - Initializes the ROS node.
    - Subscribes to the /odom topic to receive the robot's position and velocity.
    - Subscribes to the /reaching_goal/goal topic to receive the target goal position.
    - Creates a service server for the /speed_distance service to provide the distance from the target and the average speed.
    - Enters the main loop, where it accumulates the robot's speed over 50 iterations, calculates the average speed, and calculates the distance from the target.
    """
    global spd, spdf, d

    # Initialize the ROS node
    rospy.init_node('speed_node')

    # Subscribe to the odom topic to receive robot's position and velocity
    rospy.Subscriber('odom', Odometry, odom_callback)

    # Subscribe to the goal topic from the action client
    rospy.Subscriber('reaching_goal/goal', PlanningActionGoal, goal_callback)

    # Create a service server for providing distance from the target and average speed
    rospy.Service('/speed_distance', Avrg, speed_distance_callback)

    # Main loop
    rospy.spin()

if __name__ == '__main__':
    main()

