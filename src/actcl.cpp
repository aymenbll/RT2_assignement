#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <assignment_2_2023/PlanningAction.h>
#include <assignment_2_2023/Custom.h>
#include <geometry_msgs/Point.h>
#include<nav_msgs/Odometry.h>
#include <iostream>

using namespace std;
ros::Publisher pub;
double latestX = 0.0; //robot position on x coordinates
double latestY=0.0; //robot position on y coordinates
string state; 		//robot status
double od_x;
double od_y;
double od_vx;
double od_vy;


// this callback func used to get data from " /reaching_goal/feedback " which are the instantinous  values of the x and y coordinates 
void chatterCallback(const assignment_2_2023::PlanningActionFeedback::ConstPtr& feed)
{
  
  //cout << feed->feedback.actual_pose.position.x << endl;
  latestX = feed->feedback.actual_pose.position.x;
  latestY = feed->feedback.actual_pose.position.y;
  state= feed->feedback.stat;
}
// this callback function to subscribe to odom topic and get x,y coordinates and also the velocities
void chatterCallback2(const nav_msgs::Odometry::ConstPtr& od)
{
  
  
  od_x = od->pose.pose.position.x;
  od_y = od->pose.pose.position.y;
  od_vx = od->twist.twist.linear.x;
  od_vy = od->twist.twist.linear.y;

  assignment_2_2023::Custom cmsg;
  cmsg.posx=od_x;
  cmsg.posy=od_y;
  cmsg.velx=od_vx;
  cmsg.vely=od_vy;
  pub.publish(cmsg);
  

  
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "actcl");
  ros::NodeHandle n;

  // Create the action client
 
  actionlib::SimpleActionClient<assignment_2_2023::PlanningAction> ac("reaching_goal", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  assignment_2_2023::PlanningGoal goal;

  int choice, x, y;

// Create  subscribers and publisher  for  subscribing the feedback topic , subscribing the odom topic, advertising on "custom_pos_vel"topic with custom message called "Custom"
  ros::Subscriber sub = n.subscribe("reaching_goal/feedback", 10, chatterCallback);
  ros::Subscriber sub2 = n.subscribe("odom", 10, chatterCallback2);
  pub=n.advertise<assignment_2_2023::Custom> ("custom_pos_vel",1);
  


 

  while (ros::ok()) {
  	
    cout << "Enter your choice \n1- for setting target \n2- for cancel the process \n3- for getting the coordinates and the state of the robot): ";
    cin >> choice;

    if (choice == 1) {
      
      cout << "You choosed Option 1.\n";
      cout << "Enter the value of x: \n";
      cin >> x;
      cout << "Enter the value of y: \n";
      cin >> y;
      ac.waitForServer();//waiting for action server 

      goal.target_pose.pose.position.x = x; //assign the value of the target
      goal.target_pose.pose.position.y = y;

      ac.sendGoal(goal);//publish the value of the target on goal topic 
      
    } else if (choice == 2) {

      cout << "You choosed Option 2.\n";
      ac.cancelGoal();//here we cancel the process
      break;
    } else if(choice == 3) {
      //geting the value of x,y and status
      ROS_INFO("the position of X is : [%f\n]",latestX );
      ROS_INFO("the position of Y is : [%f]\n",latestY );
      ROS_INFO("the status : ");
     cout<<state<<std::endl;
      
    }
    else{
      cout<<"you choosed wrong answer";
    }


    ros::spinOnce();
  }

 // ros::spin();
  return 0;
}

