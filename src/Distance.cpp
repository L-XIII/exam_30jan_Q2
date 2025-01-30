#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <turtlesim/TeleportAbsolute.h>
#include <cstdlib>
#include <ctime>
#include <cmath>

turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;
bool turtle1_updated = false;
bool turtle2_updated = false;


void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle1_pose = *msg;
    turtle1_updated = true;
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg) {
    turtle2_pose = *msg;
    turtle2_updated = true;
}




int main(int argc, char** argv) {
    ros::init(argc, argv, "distance");
    ros::NodeHandle nh;
    
    ros::Subscriber sub1 = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber sub2 = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);
    
    // Publisher for the distance between turtles
    ros::Publisher distance_pub = nh.advertise<std_msgs::Float32>("/turtles/distance", 10);
    
    // creating Publishers in order to control turtle velocities
    ros::Publisher pub1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Publisher pub2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    
     ros::ServiceClient teleport1 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    ros::ServiceClient teleport2 = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle2/teleport_absolute");
    
    ros::Rate rate(20);  // 20 Hz
    float distance_threshold;

    if (!ros::param::has("distance_threshold")) {
        ros::param::set("distance_threshold", 2.0f);
    }
    
   while (ros::ok()) {
    ros::param::get("distance_threshold", distance_threshold);
    
    ros::spinOnce();  // Process incoming messages
    
     if (turtle1_updated && turtle2_updated) {
            // Calculate the Euclidean distance between the turtles
            float distance = std::sqrt(std::pow(turtle1_pose.x - turtle2_pose.x, 2) +
                                       std::pow(turtle1_pose.y - turtle2_pose.y, 2));
            std_msgs::Float32 distance_msg;
            distance_msg.data = distance;
            distance_pub.publish(distance_msg);

            // Check if the turtles are too close to each other or to the boundaries
            bool too_close = distance < distance_threshold;
            bool turtle1_out_of_bounds = turtle1_pose.x > 10.0 || turtle1_pose.x < 1.0 || turtle1_pose.y > 10.0 || turtle1_pose.y < 1.0;
            bool turtle2_out_of_bounds = turtle2_pose.x > 10.0 || turtle2_pose.x < 1.0 || turtle2_pose.y > 10.0 || turtle2_pose.y < 1.0;

              if (too_close || turtle1_out_of_bounds || turtle2_out_of_bounds) {
                ROS_WARN("Collision or boundary detected! Teleporting to a safe position...");

                turtlesim::TeleportAbsolute srv;
                float safe_x, safe_y;

                if (too_close || turtle1_out_of_bounds) {
                    getSafePosition(safe_x, safe_y);
                    srv.request.x = safe_x;
                    srv.request.y = safe_y;
                    srv.request.theta = 0.0;
                    teleport1.call(srv);
                }

                if (too_close || turtle2_out_of_bounds) {
                    getSafePosition(safe_x, safe_y);
                    srv.request.x = safe_x;
                    srv.request.y = safe_y;
                    srv.request.theta = 0.0;
                    teleport2.call(srv);
                }
            }
        }
        rate.sleep();
    }
    return 0;
}

   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
    
    
