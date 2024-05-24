"""
.. module:: Buggy_move
   :platform: Unix
   :synopsis: This ROS node manages robot goal setting and status monitoring by receiving user inputs, and handling feedback and odometry data.

..moduleauthor:: Aimen.B s5922996@studenti.unige.it
This node implements a controller for the Buggy

Subscribes to:
*/reaching_goal/feedback*
*/odom*

Publiches to:
*/custom_pos_vel*
""" 

#!/usr/bin/env python

import rospy
import actionlib
from assignment_2_2023.msg import PlanningAction, PlanningGoal, PlanningActionFeedback, Custom
from nav_msgs.msg import Odometry

# Global variables for robot position and status
latestX = 0.0
latestY = 0.0
state = ""
od_x = 0.0
od_y = 0.0
od_vx = 0.0
od_vy = 0.0

# Publisher for custom message
pub = None

# Callback for the "reaching_goal/feedback" topic
def feedback_callback(feed):
    """
    Callback function for the *reaching_goal/feedback* topic.
    
    Updates the robot's latest state (go to point/target reached) based on feedback from the *bug_action_service* server.

    Args:
        feed (Feedback) : Feedback message from the *bug_action_service* server containing the robot's current position, orientation and state.
    """
    global latestX, latestY, state
    latestX = feed.feedback.actual_pose.position.x
    latestY = feed.feedback.actual_pose.position.y
    state = feed.feedback.stat

# Callback for the "odom" topic
def odom_callback(od):
    """
    Callback function for the *odom* topic.
    
    Updates the robot's odometry data (position and velocity) and publishes a custom message with the current position and velocity to *custom_pos_vel* topic.

    Args:
        od (Odometry) : Odometry message containing the robot's current position and velocity.
    """
    global od_x, od_y, od_vx, od_vy
    od_x = od.pose.pose.position.x
    od_y = od.pose.pose.position.y
    od_vx = od.twist.twist.linear.x
    od_vy = od.twist.twist.linear.y

    # Publish custom message
    cmsg = Custom()
    cmsg.posx = od_x
    cmsg.posy = od_y
    cmsg.velx = od_vx
    cmsg.vely = od_vy
    pub.publish(cmsg)
    
    


def main():
    """
    Main function to initialize the ROS node, set up action client, subscribers, and publisher, and handle user input for controlling the robot.

    - Initializes the ROS node.
    - Sets up the action client for the *reaching_goal* action.
    - Subscribes to the *reaching_goal/feedback* and *odom* topics.
    - Publishes custom messages to the *custom_pos_vel* topic.
    - Handles user input to set target goals, cancel goals, and display the robot's current position and state.
    """
    global pub

    rospy.init_node('actcl')

    # Action client for "reaching_goal"
    ac = actionlib.SimpleActionClient('reaching_goal', PlanningAction)

    rospy.loginfo("Waiting for action server to start.")
    ac.wait_for_server()
    rospy.loginfo("Action server started, sending goal.")

    # Subscribers and publisher
    rospy.Subscriber("reaching_goal/feedback", PlanningActionFeedback, feedback_callback)
    """Subscribe to the *reaching_goal/feedback* topic to receive feedback on the robot's current goal status
    """
    rospy.Subscriber("odom", Odometry, odom_callback)
    """Subscribe to the *odom* topic to receive odometry data about the robot's position and velocity
    """
    pub = rospy.Publisher("custom_pos_vel", Custom, queue_size=1)
    """Publish custom position and velocity messages to the *custom_pos_vel* topic
    """

    while not rospy.is_shutdown():
        print("Enter your choice \n1- for setting target \n2- for cancel the process \n3- for getting the coordinates and the state of the robot): ")
        choice = int(input())

        if choice == 1:
            print("You chose Option 1.")
            x = float(input("Enter the value of x: "))
            y = float(input("Enter the value of y: "))

            goal = PlanningGoal()
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y

            ac.send_goal(goal)

        elif choice == 2:
            print("You chose Option 2.")
            ac.cancel_goal()
            

        elif choice == 3:
            rospy.loginfo(f"the position of X is: [{latestX}]")
            rospy.loginfo(f"the position of Y is: [{latestY}]")
            rospy.loginfo(f"the status: {state}")
            print(state)

        else:
            print("You chose the wrong answer.")

        

if __name__ == '__main__':
    main()

