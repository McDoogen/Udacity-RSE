#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

float pick_pose [2] = {-4.6, 1.0};
float drop_pose [2] = {-1.0, 5.0};


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_test");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while (ros::ok())
  {
    // Initialize and setup Marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    
    marker.type = visualization_msgs::Marker::CUBE;
    
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    
    
    
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    ROS_INFO("Publish the marker at the pickup zone");
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pick_pose[0];
    marker.pose.position.y = pick_pose[1];
    marker_pub.publish(marker);
    ros::Duration(5.0).sleep();
    
    ROS_INFO("Hide the marker");
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ros::Duration(5.0).sleep();
    
    ROS_INFO("Publish the marker at the drop off zone");
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = drop_pose[0];
    marker.pose.position.y = drop_pose[1];
    marker_pub.publish(marker);
    ros::Duration(5.0).sleep();
    
    // Repeat! :D
    r.sleep();
  }
}