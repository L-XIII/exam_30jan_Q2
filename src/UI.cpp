#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <turtlesim/Spawn.h>



int main(int argc, char** argv) {
    ros::init(argc, argv, "UI");
   
    ros::NodeHandle nh;

// We r creating  publishers for controlling the turtles (turtle1, turtle2)
// where (pub1,pub2) the nae of the publisher, (geometry_msgs::Twist) type of the maessage we will publish
    ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    
    
    
// we r creating service client to call (spawn) service
    ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");
  
//then create service request oject
    
    turtlesim::Spawn spawn_srv;
    spawn_srv.request.name = "turtle2";  // specify the name of the new turtle
    spawn_srv.request.x = 2;
    spawn_srv.request.y = 2;
    spawn_srv.request.theta = 0;
    
    // Call the spawn service directly
    if (spawnClient.call(spawn_srv)) {
        ROS_INFO("Spawned turtle %s at (%f, %f, %f)", spawn_srv.request.name.c_str(), spawn_srv.request.x,  spawn_srv.request.y, spawn_srv.request.theta);
    } 
    else {
        ROS_ERROR("Failed to call service spawn");
    }
    
    
    
    ros::Rate rate(10);  // 10 Hz
    
    while (ros::ok())
    {
    
    std::string turtle;
    float lin_vel, ang_vel;
    std::cout<<"choose turtle (1 or 2): ";
    std::cin>> turtle;
    std::cout<<"Enter linear velocity:";
    std::cin>> lin_vel;
    std::cout<<"Enter the angular velocity:";
    std::cin>> ang_vel;
    
    geometry_msgs::Twist cmd;
    cmd.linear.x = lin_vel;
    cmd.angular.z = ang_vel;

    
    if (turtle =="1")
       {
       pub1.publish(cmd);
       }
    else if (turtle=="2")
       {
       pub2.publish(cmd);
       }
    else 
       {
       std::cerr<< "Invalid turtle choice !!"<< std::endl;
       continue;
       }
       
       
    ros::Duration(1.0).sleep();
    
    cmd.linear.x = 0;
    cmd.angular.z = 0;
    pub1.publish(cmd);
    pub2.publish(cmd);
    
    
    }
    
    return 0;
    
    }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
