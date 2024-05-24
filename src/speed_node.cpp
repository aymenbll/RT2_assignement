#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2023/PlanningAction.h>
#include <assignment_2_2023/Custom.h>
#include <assignment_2_2023/Csrv.h>
#include <assignment_2_2023/Avrg.h>
#include <geometry_msgs/Point.h>
#include<nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>
using namespace std;

float dis_x;
float dis_y;
float speed_x;
float speed_y;
float final_x;
float final_y;
float dx;
float dy;
float d;
float spd=0.0;
int i;
float spdf;

//this callback function is used to get the value of instantinous position coordinates and velocity from odom topic
void chatterCallback4(const nav_msgs::Odometry::ConstPtr& od1)
{
  

  dis_x = od1->pose.pose.position.x;
  dis_y = od1->pose.pose.position.y;
  speed_x = od1->twist.twist.linear.x;
  speed_y = od1->twist.twist.linear.y;


}
//this callback function is used to get the value of last goal target choosed by the user
void chatterCallback5(const assignment_2_2023::PlanningActionGoal::ConstPtr& goal1)
{
  
  final_x = goal1->goal.target_pose.pose.position.x;
  final_y = goal1->goal.target_pose.pose.position.y;
}

bool my_random(assignment_2_2023::Avrg::Request
&req, assignment_2_2023::Avrg::Response &res){

	res.dis = d;
 	res.speed =spdf;
 	
 return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "speed_node");
  ros::NodeHandle nh;
//intialize a subscriber on odom topic
   ros::Subscriber sub = nh.subscribe("odom", 10, chatterCallback4);
//intialize a subscriber on reaching_goal/goal topic   
   ros::Subscriber sub2= nh.subscribe("reaching_goal/goal", 10, chatterCallback5);
//intialize a server that provide the average speed and the distance of the robot from the goal  the service called "/speed_distance"
    ros::ServiceServer service= nh.advertiseService("/speed_distance",my_random);
   
   while(ros::ok() ){
  
   
   for(i=0;i<50;i++){
//geting 50 sample of robot's speed and calculating the average
   spd=spd+sqrt(speed_x*speed_x+speed_y*speed_y);
   ros::spinOnce();
   
   }
    spdf=spd/i;
   i=0;
   spd=0;
//get the distance diffrence of goal and robot position on x and y 
   dx=final_x-dis_x;
   dy=final_y-dis_y;
   d=sqrt((dx*dx)+(dy*dy));
   //cout<<d;
   
   
   
   
   }

   ros::spinOnce();
  
  
  return 0;
  }

