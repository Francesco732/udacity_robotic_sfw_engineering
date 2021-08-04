#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// creat publusher object moto command
ros:: Publisher motor_command_publisher;


// callback function

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
	// ROS_INFO("DriveToTargetRequest received - j1:%1.2f, j2:%1.2f", (float).req.linear_x, (float).req.angular_z);
	while(ros::ok())
	{
		// create a motor_command objet of type geometry_msgs/Twist
		geometry_msgs::Twist motor_command;
		motor_command.linear.x = (float)req.linear_x;
		motor_command.angular.z = (float)req.angular_z;

		motor_command_publisher.publish(motor_command);
	
	}

	res.msg_feedback = "linear velocity x: " + std::to_string((float)req.linear_x) + ", angular velocity z: " + std::to_string((float)req.angular_z);
	ROS_INFO_STREAM(res.msg_feedback);
	return true;

}


int main(int argc, char** argv)
{
	// initialize a ROS node
	ros::init(argc, argv, "drive_bot");

	// create a ROS NodeHandle object
	ros::NodeHandle n;

	// inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic, with queue of 10
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

	// define a drive /ball_chaser/command_robot service with handle_drive_request callback function
	ros::ServiceServer service=n.advertiseService("/ball_chaser/command_robot", handle_drive_request);


	// handle ROS communication events
	ros::spin();



	return 0;




}
