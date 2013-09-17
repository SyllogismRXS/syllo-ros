#ifndef ROBOT_H_
#define ROBOT_H_
/// ----------------------------------------------------------------------------
/// @file Robot.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-09-17 18:04:41 syllogismrxs>
///
/// @version 1.0
/// Created: 17 Sep 2013
///
/// ----------------------------------------------------------------------------
/// @section LICENSE
/// 
/// The MIT License (MIT)  
/// Copyright (c) 2012 Kevin DeMarco
///
/// Permission is hereby granted, free of charge, to any person obtaining a 
/// copy of this software and associated documentation files (the "Software"), 
/// to deal in the Software without restriction, including without limitation 
/// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
/// and/or sell copies of the Software, and to permit persons to whom the 
/// Software is furnished to do so, subject to the following conditions:
/// 
/// The above copyright notice and this permission notice shall be included in 
/// all copies or substantial portions of the Software.
/// 
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
/// DEALINGS IN THE SOFTWARE.
/// ----------------------------------------------------------------------------
/// @section DESCRIPTION
/// 
/// The Robot class ...
/// 
/// ----------------------------------------------------------------------------

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Stage-4.1/stage.hh>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

using std::cout;
using std::endl;

class Robot
{
private:
protected:
     std::string name_;
     
     std::string odom_topic_;
     std::string gt_odom_topic_;
     std::string cmd_vel_topic_;

     ros::NodeHandle *n_;
     ros::Publisher odom_pub_;
     ros::Publisher gt_odom_pub_;
     ros::Subscriber cmd_vel_sub_; 

     nav_msgs::Odometry odom_;
     nav_msgs::Odometry gt_odom_;
     geometry_msgs::Twist cmd_vel_;

public:
     Stg::ModelPosition* position;
     Stg::ModelRanger* ranger;

     void callback_cmd_vel(const geometry_msgs::TwistConstPtr& msg)
          {
               cout << name_ << " X: " << msg->linear.x;
               cmd_vel_ = *msg;
          }

     void set_name(const std::string &name)
          {
               name_ = name;
          }
     
     void RegisterTopics(ros::NodeHandle *n)
          {
               n_ = n;
               
               odom_topic_ = "/" + name_ + "/odom";
               gt_odom_topic_ = "/" + name_ + "/ground_truth_odom";
               
               odom_pub_ = n_->advertise<nav_msgs::Odometry>(odom_topic_,1);
               gt_odom_pub_ = n_->advertise<nav_msgs::Odometry>(gt_odom_topic_,1);

               cmd_vel_topic_ = "/" + name_ + "/cmd_vel";
               cmd_vel_sub_ = n->subscribe(cmd_vel_topic_, 1, &Robot::callback_cmd_vel, this);
          }

     geometry_msgs::Twist & cmd_vel()
          {
               return cmd_vel_;
          }

     void publish_odometry()
          {
               odom_.header.stamp = ros::Time::now();
               
               odom_.pose.pose.position.x = position->est_pose.x;
               odom_.pose.pose.position.y = position->est_pose.y;
               odom_.pose.pose.position.z = position->est_pose.z;
               odom_pub_.publish(odom_);

               Stg::Pose global_pose = position->GetGlobalPose();
               gt_odom_.pose.pose.position.x = global_pose.x;
               gt_odom_.pose.pose.position.y = global_pose.y;
               gt_odom_.pose.pose.position.z = global_pose.z;
               gt_odom_pub_.publish(gt_odom_);               
          }
};

#endif
