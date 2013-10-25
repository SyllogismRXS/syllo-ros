#include <iostream>

#include <syllo_common/SylloNode.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <sstream>

using std::cout;
using std::endl;

nav_msgs::Odometry gt_;

void ground_truth_callback(const nav_msgs::OdometryConstPtr& msg)
{
     gt_ = *msg;
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "trajectory");

     ros::NodeHandle n;

     SylloNode syllo_node_;
     syllo_node_.init();

     double goal_position;
     ros::param::param<double>("goal_position", goal_position, 0);     

     ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

     ros::Subscriber gt_sub = n.subscribe("ground_truth_odom", 1, ground_truth_callback);

     geometry_msgs::Twist cmd_vel_;
     cmd_vel_.linear.x = 1;
     cmd_vel_.angular.z = 0;

     while (ros::ok()) {
          cmd_vel_pub.publish(cmd_vel_);
          syllo_node_.spin();
     }

     // Clean up syllo node
     syllo_node_.cleanup();

     return 0;
}
