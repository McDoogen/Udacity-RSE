#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

float pick_pose [2] = {-4.6, 1.0};
float drop_pose [2] = {-1.0, 5.0};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  /*	Pick up Goal	*/
  move_base_msgs::MoveBaseGoal pickGoal;

  // set up the frame parameters
  pickGoal.target_pose.header.frame_id = "map";
  pickGoal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the pick up goal
  pickGoal.target_pose.pose.position.x = pick_pose[0];
  pickGoal.target_pose.pose.position.y = pick_pose[1];
  pickGoal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(pickGoal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Yay! The robot reach it's pick-up goal!!");
  else
    ROS_INFO("Oh no! The robot failed to reach the pick-up goal!");
  
  // Wait for 5 seconds
  ros::Duration(5.0).sleep();
  
  
  /*	Drop off Goal	*/
  move_base_msgs::MoveBaseGoal dropGoal;

  // set up the frame parameters
  dropGoal.target_pose.header.frame_id = "map";
  dropGoal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the drop-off goal
  dropGoal.target_pose.pose.position.x = drop_pose[0];
  dropGoal.target_pose.pose.position.y = drop_pose[1];
  dropGoal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(dropGoal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Yay! The robot reach it's drop-off goal!!");
  else
    ROS_INFO("Oh no! The robot failed to reach the drop-off goal!");
  
  

  return 0;
}