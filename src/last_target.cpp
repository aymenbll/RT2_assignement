#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2023/PlanningAction.h>
#include <assignment_2_2023/Custom.h>
#include <assignment_2_2023/Csrv.h>
#include <geometry_msgs/Point.h>
#include<nav_msgs/Odometry.h>
#include <iostream>

float target_x;//value of goal's x coordinates
float target_y;//value of goal's y coordinates

//this callback function is used to get the value of last goal target choosed by the user
void chatterCallback3(const assignment_2_2023::PlanningActionGoal::ConstPtr& goal)
{
  

  target_x = goal->goal.target_pose.pose.position.x;
  target_y = goal->goal.target_pose.pose.position.y;
}

bool my_random(assignment_2_2023::Csrv::Request
&req, assignment_2_2023::Csrv::Response &res){

	res.goal_x = target_x;
 	res.goal_y = target_y;
 	
 return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "last_target");
  ros::NodeHandle nh;

//advertising the value of the last target on the service "/Last_Target"
   ros::ServiceServer service= nh.advertiseService("/Last_Target",my_random);
//subscribing to /reaching_goal/goal topic
   ros::Subscriber sub = nh.subscribe("reaching_goal/goal", 10, chatterCallback3);


  ros::spin();
  
  
  return 0;
  }
  
