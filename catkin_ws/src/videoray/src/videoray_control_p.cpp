#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "videoray/Throttle.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <sstream>
#include <cmath>

using std::cout;
using std::endl;

//
// Converts desired heading, desired velocity, and desired depth into
// throttle (left, right, vertical) commands
//

double desired_velocity_ = 0;
double desired_heading_ = 0;
double desired_depth_ = 0;

double roll_ = 0;
double pitch_ = 0;
double yaw_ = 0;

videoray::Throttle throttle_;
geometry_msgs::Pose pose_;
geometry_msgs::Twist velocity_;


void desiredVelocityCallback(const std_msgs::Float32::ConstPtr& msg)
{
     desired_velocity_ = msg->data;
}

void desiredHeadingCallback(const std_msgs::Float32::ConstPtr& msg)
{
     desired_heading_ = msg->data;
}

void desiredDepthCallback(const std_msgs::Float32::ConstPtr& msg)
{
     desired_depth_ = msg->data;
     ROS_INFO("Desired Depth: %f", desired_depth_);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
     pose_ = msg->pose;
}

void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
     velocity_ = *msg;
}

void quaternionToEuler(const double &q0, const double &q1, 
                       const double &q2, const double &q3,
                       double &roll, double &pitch, double &yaw)
{
     roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2) );
     pitch = asin(2*(q0*q2-q3*q1));
     yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3) );
}

double normDegrees(double input)
{
     if (input < 0) {
          input += 360;
     } else if(input >= 360) {
          input -= 360;
     }
     return input;
}

#define PI (3.14159265359)

int main(int argc, char **argv)
{
     ros::init(argc, argv, "videoray_control_p");
     
     ros::NodeHandle n;

     ros::Publisher throttle_pub = 
          n.advertise<videoray::Throttle>("throttle_cmds", 1000);
     
     ros::Subscriber desired_vel_sub = n.subscribe("desired_velocity", 
                                                   1000, 
                                                   desiredVelocityCallback);

     ros::Subscriber desired_head_sub = n.subscribe("desired_heading", 
                                                    1000, 
                                                    desiredHeadingCallback);

     ros::Subscriber desired_depth_sub = n.subscribe("desired_depth", 
                                                     1000, 
                                                     desiredDepthCallback);

     ros::Subscriber pose_sub = n.subscribe("pose", 
                                            1000, 
                                            poseCallback);

     ros::Subscriber morse_vel_sub = n.subscribe("actual_velocity", 
                                                 1000, 
                                                 velocityCallback);



     ros::Rate loop_rate(10);

     while (ros::ok())
     {

          // Depth Proportional Controller:
          double depth_err = 5*(desired_depth_ - pose_.position.z);
          throttle_.VerticalThrottle = depth_err;

          // Heading controller:
          geometry_msgs::Quaternion quat = pose_.orientation;

          //quaternionToEuler(quat.x, quat.y, quat.z, quat.w,
          //                  roll_, pitch_, yaw_);
          quaternionToEuler(quat.w, quat.x, quat.y, quat.z,
                            roll_, pitch_, yaw_);

          ROS_INFO("Desired heading: %f", desired_heading_);
          ROS_INFO("Actual heading: %f", normDegrees(yaw_*180/PI));

          yaw_ = normDegrees(yaw_*180.0/PI);


          // fix heading first:
          double heading_err = (desired_heading_ - yaw_);

          if (abs(heading_err) < 180) {
               throttle_.LeftThrottle = -heading_err;
               throttle_.RightThrottle = heading_err;
          } else {
               throttle_.LeftThrottle = heading_err;
               throttle_.RightThrottle = -heading_err;
          }
          
          // Velocity controller (forward direction: x:
          //double vel_err = (desired_velocity_ - velocity_.linear.x);
          //double vel_err = desired_velocity_;
          //throttle_.LeftThrottle = vel_err;
          //throttle_.RightThrottle = vel_err;

          throttle_pub.publish(throttle_);

          ros::spinOnce();
          loop_rate.sleep();
     }
     return 0;
}
