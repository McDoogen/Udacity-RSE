#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction

void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
  	srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if(!client.call(srv)) {
		ROS_ERROR("Failed to call service DriveToTarget");
    }
}



// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
	int left_count = 0;
	int middle_count = 0;
	int right_count = 0;
    
    float linear_speed = 0.1;
    float angular_speed = 0.2;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    // Search for white Pixels
    for(int i=0; i<img.step * img.height; i+=3) {
		if((int)img.data[i] == white_pixel &&
		(int)img.data[i+1] == white_pixel &&
		(int)img.data[i+2] == white_pixel) {
			if (i/3 % img.height < img.width/3)
				left_count++;
			else if (i/3 % img.height < 2*(img.width/3))
				middle_count++;
			else
				right_count++;
		}
	}
	
	// Determine direction to drive
	ROS_INFO_STREAM(left_count);
	ROS_INFO_STREAM(middle_count);
	ROS_INFO_STREAM(right_count);
	if(!left_count && !middle_count && !right_count) {
		// Stop if nothing is found
       	ROS_INFO_STREAM("Stop");
		drive_robot(0.0, 0.0);
	}
	else if(left_count > middle_count && left_count > right_count) {
		// Drive left if 'left_count' is greatest
		ROS_INFO_STREAM("Drive Left");
		drive_robot(linear_speed, angular_speed);
	}
	else if(middle_count > right_count) {
		// Drive straight if 'middle_count' is greatest
		ROS_INFO_STREAM("Drive Straight");
		drive_robot(linear_speed, 0.0);
	}
	else {
		// Drive right if 'right_count' is greatest
		ROS_INFO_STREAM("Drive Right");
		drive_robot(linear_speed, -angular_speed);
	}
}

  
int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);
  
    // Handle ROS communication events
    ros::spin();

    return 0;
}



