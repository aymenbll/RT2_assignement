"""
.. module:: Last_Target
   :platform: Unix
   :synopsis: This ROS node manages the retrieval of the last target goal coordinates by subscribing to the 'reaching_goal/goal' topic and providing the coordinates through a service.

.. moduleauthor:: Aimen.B s5922996@studenti.unige.it

This node implements a service to retrieve the last goal target coordinates.

Subscribes to:
    * /reaching_goal/goal

Provides Service:
    * /Last_Target
"""

#!/usr/bin/env python

import rospy
from assignment_2_2023.msg import PlanningActionGoal
from assignment_2_2023.srv import Csrv, CsrvResponse

# Global variables to store the last target goal coordinates
target_x = 0.0
target_y = 0.0

# Callback function to update the last goal target coordinates
def goal_callback(goal):
    """
    Callback function for the *reaching_goal/goal* topic.
    
    Updates the global variables with the latest goal target coordinates.

    Args:
        goal (PlanningActionGoal): Goal message containing the target coordinates.
    """
    global target_x, target_y
    target_x = goal.goal.target_pose.pose.position.x
    target_y = goal.goal.target_pose.pose.position.y

# Service handler function to return the last goal target coordinates
def handle_last_target(req):
    """
    Service handler function for the *Last_Target* service.
    
    Returns the last goal target coordinates.

    Args:
        req (CsrvRequest): Service request.

    Returns:
        CsrvResponse: Service response containing the last goal target coordinates.
    """
    res = CsrvResponse()
    res.goal_x = target_x
    res.goal_y = target_y
    return res

def main():
    """
    Main function to initialize the ROS node, advertise the service, and subscribe to the 'reaching_goal/goal' topic.

    - Initializes the ROS node.
    - Advertises the *Last_Target* service.
    - Subscribes to the *reaching_goal/goal* topic to get the latest goal target coordinates.
    """
    rospy.init_node('last_target')
  

    # Advertise the service to provide the last target coordinates
    service = rospy.Service('/Last_Target', Csrv, handle_last_target)
    """Provides the *Last_Target* service which returns the last goal target coordinates.
    """    
    # Subscribe to the "reaching_goal/goal" topic to get the latest goal target
    rospy.Subscriber("reaching_goal/goal", PlanningActionGoal, goal_callback)
    """Subscribe to the *reaching_goal/goal* topic to receive the latest goal target coordinates.
    """

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    main()

