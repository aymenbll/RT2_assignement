#!/usr/bin/env python

"""
.. module:: Display_speed
   :platform: Unix
   :synopsis: This ROS node manages robot goal setting and status monitoring by receiving user inputs, and handling feedback and odometry data.

.. moduleauthor:: Aimen.B s5922996@studenti.unige.it
   This node implements a controller for the Buggy.

Subscribes to:
    /custom_pos_vel (my_robot_control.msg.PosVel)
        - Receives the current position and velocity of the robot.
    /reaching_goal/goal (assignment_2_2023.msg.PlanningActionGoal)
        - Receives the goal position from the action client.

Publishes to:
    None

Services:
    /distance_avgspeed (my_robot_control.srv.DisSpd)
        - Provides the distance to the target and the average speed of the robot.
"""

import rospy
import math
from assignment_2_2023.msg import PlanningActionGoal
from my_robot_control.msg import PosVel
from my_robot_control.srv import DisSpd, DisSpdResponse

# Global variables to store robot's position, velocity, target coordinates, and calculated distance and speed
pos_x = 0.0
pos_y = 0.0
vel_x = 0.0
target_x = 0.0
target_y = 0.0
d = 0.0
spd = 0.0
spdf = 0.0

def callback1(custom):
    """
    Callback function for receiving robot position and velocity.

    Args:
        custom (my_robot_control.msg.PosVel): Custom message containing robot's position and velocity.
    """
    global pos_x, pos_y, vel_x
    pos_x = custom.pos_x
    pos_y = custom.pos_y
    vel_x = custom.vel_x

def callback2(goal1):
    """
    Callback function for receiving the goal from the action client.

    Args:
        goal1 (assignment_2_2023.msg.PlanningActionGoal): Goal message from the action client.
    """
    global target_x, target_y
    target_x = goal1.goal.target_pose.pose.position.x
    target_y = goal1.goal.target_pose.pose.position.y

def callback3(req):
    """
    Service callback function to provide distance from the target and average speed.

    Args:
        req (my_robot_control.srv.DisSpdRequest): Service request.

    Returns:
        my_robot_control.srv.DisSpdResponse: Service response containing distance and average speed.
    """
    res = DisSpdResponse()
    res.dis = d
    res.speed = spdf
    return res

def main():
    """
    Main function to initialize the ROS node, subscribers, and service server.
    The main function performs the following steps:
    1. Initializes the ROS node.
    2. Subscribes to the /custom_pos_vel topic to receive the robot's position and velocity.
    3. Subscribes to the /reaching_goal/goal topic to receive the target goal position.
    4. Creates a service server for the /distance_avgspeed service to provide the distance from the target and the average speed.
    5. Enters the main loop:
        a. Accumulates the robot's speed over 10 iterations, updating the global `spd` variable.
        b. Calculates the average speed (`spdf`) over these iterations.
        c. Resets the speed accumulator (`spd`) for the next cycle.
        d. Calculates the Euclidean distance (`d`) from the current position to the target position.
    6. Calls `rospy.spin()` once to handle any callbacks.
    """
    global spd, spdf, d

    # Initialize the ROS node
    rospy.init_node('dis_speed_node', anonymous=True)

    # Subscribe to robot's position and velocity topic
    rospy.Subscriber('/custom_pos_vel', PosVel, callback1)

    # Subscribe to the goal topic from the action client
    rospy.Subscriber('reaching_goal/goal', PlanningActionGoal, callback2)

    # Create a service server for providing distance from the target and average speed
    rospy.Service('/distance_avgspeed', DisSpd, callback3)

    # Main loop
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Accumulate speed over a period of time
        for _ in range(10):
            spd += vel_x
            rospy.sleep(0.1)
        
        # Calculate average speed
        spdf = spd / 10.0
        spd = 0.0
        
        # Calculate distance from the target using Euclidean distance formula
        dx = target_x - pos_x
        dy = target_y - pos_y
        d = math.sqrt(dx**2 + dy**2)

        # Call spin once to handle callbacks
        rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

