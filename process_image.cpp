#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in te specific direction
void drive_robot(float lin_x, float ang_z)
{
	ROS_INFO_STREAM("Move the robot");
	// I request a service and pass the velocities to it to drive the robot
	
	// define a request to service:
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	// call command_robot service
	//client.call(srv);
	if(!client.call(srv)){
	   ROS_ERROR("Failed to call command_robot service");
	}

}

// This callback function determine the position executing and reading image data

void process_image_callback(const sensor_msgs::Image img)
{
	int white_pixel = 255;
	ROS_INFO_STREAM("The width of image is: " + std::to_string(img.width));
	bool ball_in_camera = false;
	// Loop through each pixel in the image and check where are white pixels
	const int left_treshold = 250;
	const int right_treshold = 650;
	int location = 0;

	for (int i = 0; i < img.height * img.step; i+=3){
	    if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel){
			ball_in_camera = true;
			location = i % img.width;
			break;
	    }
	}

	if (ball_in_camera ==true){
		ROS_INFO_STREAM("Ball found in camera location " + std::to_string(location));
		if (location < left_treshold){
		    // flag left
		    ROS_INFO_STREAM("LEFT");
		    drive_robot(0.0, 0.5);
		}
			    
		else if (location > left_treshold && location < right_treshold){
		    // flag forward
		    ROS_INFO_STREAM("FORWARD");
		    drive_robot(0.5, 0.0);				
		}
		else if (location >= right_treshold && location < img.height*img.step){
		    // flag right
		    ROS_INFO_STREAM("RIGHT");
		    drive_robot(0.0, -0.5);	
		}
	}else {
	    // flag stop
	    ball_in_camera = false;
	    ROS_INFO_STREAM("STOP");
	    drive_robot(0.0, 0.0);
	}
}

// main

int main(int argc, char** argv)
{

	// Initializate the process_image node and create a handle to it
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	// Definie a client service capable of requesting services from command_robot, (the name of the service defined in drive_bot)
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	// Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
	// The data taken from this topic are passed as argument to the callback function
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	// Handle ROS communication events
	ros::spin();
	return 0;

}
