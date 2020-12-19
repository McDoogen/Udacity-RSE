#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

bool at_pick_up = false;
bool at_drop_off = false;
float pick_pose [2] = {-4.6, 1.0};
float drop_pose [2] = {-1.0, 5.0};
float distance_error = 0.6;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  //Track distance to goals
  if(!at_pick_up) {
    float d = sqrt(  pow(msg->pose.pose.position.y - pick_pose[0],2) + pow(-msg->pose.pose.position.x - pick_pose[1],2)  );
    ROS_INFO("Distance to pick-up is [%f]", d);
    ros::Duration(1.0).sleep();
    
    if(d < distance_error) {
      at_pick_up = true;
      ROS_INFO("Picking up marker");
    }
  }
  else if(!at_drop_off) {
    float d = sqrt(  pow(msg->pose.pose.position.y - drop_pose[0],2) + pow(-msg->pose.pose.position.x - drop_pose[1],2)  );
    ROS_INFO("Distance to drop-off is [%f]", d);
    ros::Duration(1.0).sleep();
    
    if(d < distance_error) {
      at_drop_off = true;
      ROS_INFO("Dropping off marker");
    }
  }
  else {
    ROS_INFO("YAY! We dropped off the marker!!! :D");
  }
  
  
  
  // Publish Marker positions
  if(!at_pick_up) {
    ROS_INFO("Not at Pick-up yet");
    marker.pose.position.x = pick_pose[0];
    marker.pose.position.y = pick_pose[1];
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
  }
  else if(at_drop_off) {
    ROS_INFO("At drop-off! :D");
    marker.pose.position.x = drop_pose[0];
    marker.pose.position.y = drop_pose[1];
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
  }
  else {
    ROS_INFO("Not at drop-off yet");
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);

  // Initialize and setup Marker
  //visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type, action, pose, size, and color.
  marker.type = visualization_msgs::Marker::CUBE;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  
  // Loop forever and ever! :D
  ros::spin();
}