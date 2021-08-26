#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Definine a global client that can request a service
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in  the specific direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass te velocities to it to drive the robot
    // ROS_INFO_STREAM("Move the robot");
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // call the drive_bot command_robot service and pass requester velocities to it
    if (!client.call(srv))
        ROS_ERROR("Failed to call the service command_robot");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 225;
    bool ball_in_camera = false;
    int left_treshold = (img.step)/3;
    int right_treshold = (img.step)*2/3;
    int location = 0;

    // one way to detect the ball
    //for (int row = 0; row < img.height; row++){
    //    int width_step = img.step * row;
    //    for (int col = 0; col < img.step; col +=3){
    //        if (img.data[width_step+col] == white_pixel && img.data[width_step+col+1] == white_pixel && img.data[width_step+col+2]==white_pixel){
    //            ball_in_camera = true;
    //            location = col/3;
    //            ROS_INFO_STREAM("Ball detected");
    //            break; 
    //        }
    //    }
    //    if (ball_in_camera == true){
    //        break;
    //    }
    //}

    // second way to detect the ball
    for (int i = 0; i < img.height*img.step; i++){
        // ROS_INFO_STREAM("Pixel value: " + std::to_string(img.data[i]));
        if (img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2]==white_pixel){
            ball_in_camera = true;
            location = i % img.step;
            ROS_INFO_STREAM("Ball detected");
            break;
        }
    }

    if (ball_in_camera == true){
        if (location < left_treshold){
            // Move to the left
            //ROS_INFO_STREAM("Location value: " + std::to_string(location) + " so move left");
            drive_robot(0, 0.5);
        }
        if (location > left_treshold && location < right_treshold){
            // Move forward
            //ROS_INFO_STREAM("Location value: " + std::to_string(location) + " so move forward");
            drive_robot(0.5,0);
        }
        if (location > right_treshold){
            // Move right
            //ROS_INFO_STREAM("Location value: " + std::to_string(location) + " so move right");
            drive_robot(0,-0.5);
        }
    }else{
        // Stop
        ROS_INFO_STREAM("No ball in camera, the robot do not move");
        drive_robot(0,0);
    }

}

int main (int argc, char** argv)
{
    // Initializate the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting
    // service from command_robot service
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    
    // Subscribe to /camera/rgb/image_raw topic to read the image
    // data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;

}