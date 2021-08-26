#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// Ros publisher motor command
ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("DriveToTarget request received - j1:%1.2f, j2:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // Publish the data requested

    // I create an object motor_command of type geometry_msgs::Twist that allows me to set the wheel velocity at the requestet velocities
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = (float)req.linear_x;
    motor_command.angular.z = (float)req.angular_z;

    // Now I publish the requested data
    motor_command_publisher.publish(motor_command);

    return true;

}

int main(int argc, char** argv)
{
    // Initializate ros node drive_bot
    ros::init(argc, argv, "drive_bot");

    // Create a Ros NodeHandle n
    ros::NodeHandle n;

    // Inform Ros that we want to publish a message of type geometry_msgs::Twist on topic /cmd_vel
    // with que size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Define a drive ball_chaser/command_robot servicewith a handle_drive_request call back function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    // ROS_INFO_STREAM("Ready to send joints commands");

    // Handle Ros communication event
    ros::spin();

    return 0;

}