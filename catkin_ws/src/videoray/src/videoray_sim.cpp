#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "videoray/Throttle.h"

#include <iostream>
#include <sstream>

#include <boost/numeric/odeint.hpp>

using std::cout;
using std::endl;

using namespace boost::numeric::odeint;

typedef boost::array< double , 6 > state_type;

#define PI (3.14159265359)

double sigma = 135 * PI / 180;
//double sigma = 170 * PI / 180;
double F_r = 0;
double F_l = 0;
double r = 1;
//double r = 5;

void videoray_model( const state_type &x , state_type &dxdt , double t )
{

     dxdt[0] = F_r * cos(x[3]) + F_l * sin(x[3]);
     dxdt[1] = F_r * sin(x[3]) + F_l * cos(x[3]);
     dxdt[2] = r * F_r * sin(sigma) + r * F_l * cos(sigma);

     dxdt[3] = 20*x[0];
     dxdt[4] = 20*x[1];
     dxdt[5] = 20*x[2];

     //dxdt[0] = sigma * ( x[1] - x[0] );
     //dxdt[1] = R * x[0] - x[1] - x[0] * x[2];
     //dxdt[2] = -b * x[2] + x[0] * x[1];
}

//
// Converts input throttle commands to simulated linear and angular velocities.
//
geometry_msgs::Twist twist;
geometry_msgs::Vector3 linear;
geometry_msgs::Vector3 angular;

geometry_msgs::Twist acceleration_;
//geometry_msgs::Twist velocity_;

geometry_msgs::Pose pose_;
double roll_ = 0;
double pitch_ = 0;
double yaw_ = 0;

double throttleToVelocity(double throttle)
{    
     return 0;
}

double saturate(double input, const double &min, const double &max)
{
     if (min > max) {
          ROS_INFO("saturate(): Invalid Min / Max Combo");
          return 0;
     } else if (input < min) {
          input = min;
     } else if(input > max) {
          input = max;
     }
     return input;
}

// Assumes that input has already been saturated within the in_min and in_max
// boundaries. Use the saturate() function on input before calling normalize
double normalize(double input, const double &in_min, const double &in_max,
                 const double &out_min, const double &out_max)
{
     if (in_min >= in_max || out_min >= out_max) {
          ROS_INFO("normalize(): Invalid Min / Max Combo");
          return 0;
     }

     double ratio = input / (in_max - in_min);
     return ratio * (out_max - out_min);

     return input;
}

void quaternionToEuler(const double &q0, const double &q1, 
                       const double &q2, const double &q3,
                       double &roll, double &pitch, double &yaw)
{
     roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2) );
     pitch = asin(2*(q0*q2-q3*q1));
     yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3) );
}

double radius = 0.5;
double length = 1;

runge_kutta4< state_type > stepper; 

double left_vel_ = 0;
double right_vel_ = 0;
double vert_vel_ = 0;
void throttleCallback(const videoray::Throttle::ConstPtr& msg)
{
     // Left Throttle Conversion:
     left_vel_ = saturate(msg->LeftThrottle, -100, 100);
     left_vel_ = normalize(left_vel_, -100, 100, -1, 1);
     //linear.x = left_vel;
     //angular.z = left_vel / 3;

     // Right Throttle Conversion:
     right_vel_ = saturate(msg->RightThrottle, -100, 100);
     right_vel_ = normalize(right_vel_, -100, 100, -1, 1);
     //linear.x += right_vel_;
     //angular.z -= right_vel / 3;    

     //geometry_msgs::Quaternion quat = pose_.orientation;
     //quaternionToEuler(quat.x, quat.y, quat.z, quat.w,
     //                  roll_, pitch_, yaw_);
     
     //// accel:x, accel:y, accel:theta, veloc:x, veloc:y, veloc:theta
     //state_type x = {acceleration_.linear.x, acceleration_.linear.y,
     //                acceleration_.angular.z, 
     //                velocity_.linear.x, velocity_.linear.y,
     //                velocity_.angular.z};
     
     // accel:x, accel:y, accel:theta, veloc:x, veloc:y, veloc:theta
     // as of now, MORSE stuffs linear accelerometer VELOCITIES into the
     // accelerometer TWIST packet
     //state_type x = {0,0,0,
     //                acceleration_.linear.x, acceleration_.linear.y,
     //                0};
     //
     ////stepper.do_step(videoray_model, x , t , dt ); 
     //stepper.do_step(videoray_model, x , 0 , dt ); 


     //linear.x = radius/2.0 * (left_vel + right_vel) * cos(yaw_); 
     //linear.y = radius/2.0 * (left_vel + right_vel) * sin(yaw_); 
     //linear.x = left_vel + right_vel;
     //linear.x = left_vel;
     

     // Vertical Throttle Conversion:
     vert_vel_ = saturate(msg->VerticalThrottle, -100, 100);
     vert_vel_ = normalize(vert_vel_, -100, 100, -1, 1);
     //linear.z = vert_vel_;
     
     // Rotation:
     //angular.z = radius / length * (right_vel - left_vel);

     //// Left Rotational velocity
     //double left_rot_vel = -1*left_vel*sin(130*PI/180);
     //double right_rot_vel = 1*right_vel*sin(130*PI/180);
     //angular.z = left_rot_vel + right_rot_vel;


}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
     pose_ = msg->pose;
}

void accelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
     acceleration_ = *msg;
}


//void velocityCallback(const geometry_msgs::Twist::ConstPtr& msg)
//{
//     velocity_ = *msg;
//}


int main(int argc, char **argv)
{
     ros::init(argc, argv, "videoray_sim");
     
     ros::NodeHandle n;

     ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("motion", 1000);
     ros::Subscriber throttle_sub = n.subscribe("throttle_cmds", 1000, 
                                                throttleCallback);

     ros::Subscriber pose_sub = n.subscribe("pose", 
                                            1000, 
                                            poseCallback);
     
     ros::Subscriber accel_sub = n.subscribe("acceleration", 
                                            1000, 
                                            accelCallback);

     //ros::Subscriber vel_sub = n.subscribe("velocity", 
     //                                       1000, 
     //                                       velocityCallback);

     ros::Rate loop_rate(10);

     while (ros::ok())
     {
          //linear.x = 0.1; //forward
          //linear.z = 0.1; // up
          //          
          //angular.x = 0.0; //rotate about x axis
          //angular.y = 0.0;
          //angular.z = -0.1; // rotate right
          //

          geometry_msgs::Quaternion quat = pose_.orientation;
          quaternionToEuler(quat.x, quat.y, quat.z, quat.w,
                            roll_, pitch_, yaw_);
          
          // accel:x, accel:y, accel:theta, veloc:x, veloc:y, veloc:theta
          // as of now, MORSE stuffs linear accelerometer VELOCITIES into the
          // accelerometer TWIST packet
          state_type x = {0,0,0,
                          acceleration_.linear.x, acceleration_.linear.y,
                          0};

          //stepper.do_step(videoray_model, x , t , dt ); 
          F_l = left_vel_;
          F_r = right_vel_;
          stepper.do_step(videoray_model, x , 0 , 1.0/10.0 ); 
          stepper.do_step(videoray_model, x , 0 , 1.0/10.0 ); 
          
          linear.x = x[3];
          linear.y = x[4];
          angular.z = x[5];

          linear.z = vert_vel_;

          twist.linear = linear;
          twist.angular = angular;
          twist_pub.publish(twist);

          ros::spinOnce();

          loop_rate.sleep();
     }
     

     return 0;
}
