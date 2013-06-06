#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "videoray/Throttle.h"

#include <iostream>
#include <sstream>

//#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>

using std::cout;
using std::endl;

using namespace boost::numeric::odeint;

//typedef boost::array< double , 6 > state_type;
typedef boost::array< double , 12 > state_type;

#define PI (3.14159265359)

double sigma = 135 * PI / 180;
//double sigma = 170 * PI / 180;
double F_r = 0;
double F_l = 0;
double r = 1;
//double r = 5;

state_type x_ = {0,0,0,0,0,0,0,0,0,0,0,0};

void videoray_model( const state_type &x , state_type &dxdt , double t )
{
/// States: 
/// 0:  u     : surge velocity
/// 1:  v     : sway velocity
/// 2:  w     : heave velocity
/// 3:  p     : roll rate
/// 4:  q     : pitch rate
/// 5:  r     : yaw rate
/// 6:  xpos  : earth x-pos
/// 7:  ypos  : earth y-pos
/// 8:  zpos  : earth z-pos
/// 9: phi   : roll angle
/// 10: theta : pitch angle
/// 11: psi   : yaw angle
/// 12: Thrust_X : Forward thrust
/// 13: Thrust_Theta : Yaw Thrust
/// 14: Thrust_Z : Vertical Thrust

     double u     = x[0];
     double v     = x[1];
     double w     = x[2];
     double p     = x[3];
     double q     = x[4];
     double r     = x[5];
     double xpos  = x[6];
     double ypos  = x[7];
     double zpos  = x[8];
     double phi   = x[9];
     double theta = x[10];
     double psi   = x[11];
     
     double X_udot = 1.94; // inertia matrix M (m11)
     double Y_vdot = 6.05; // inertia matrix M (m22)
     double N_rdot = 0.5;  // [TODO] N_rdot = 1.18e-2; // vehicle's motion of inertia about z-axis
     // (6,6) entry of the vehicle inertia Matrix M

     double Xu = -0.95;
     double Xuu = -6.04;
     double Yv = -5.87;
     double Yvv = -30.73;
     double Nr = -0.023;
     double Nrr = -0.45;

     // Control inputs
     double X = 0.3;
     double N = 0.1;
     double Z = 0.3;

     dxdt[0] = (-Y_vdot*v*r + Xu*u + Xuu*u*abs(u) + X) / X_udot;
     dxdt[1] = (X_udot*u*r + Yv*v + Yvv*v*abs(v)) / Y_vdot;
     
     // Heave
     //Z_wdot = 3.95; //m33
     double Z_wdot = 9; //m33 [TODO]
     double Zw = -3.70;
     double Zww = -26.36;     
     dxdt[2] = (Zw*w + Zww*w*abs(w) + Z) / Z_wdot;

     // Rotations
     dxdt[3] = 0;
     dxdt[4] = 0;
     dxdt[5] = (Nr*r + Nrr*r*abs(r) + N) / N_rdot;

     //Precalculate trig functions
     double c1 = cos(phi);
     double c2 = cos(theta); 
     double c3 = cos(psi); 
     double s1 = sin(phi); 
     double s2 = sin(theta); 
     double s3 = sin(psi); 
     double t2 = tan(theta);

     //Calculate Positions
     dxdt[6] = c3*c2*u + (c3*s2*s1-s3*c1)*v + (s3*s1+c3*c1*s2)*w;
     dxdt[7] = s3*c2*u + (c1*c3+s1*s2*s3)*v + (c1*s2*s3-c3*s1)*w;
     dxdt[8] = -s2*u + c2*s1*v + c1*c2*w;

     //Calculate Angles
     dxdt[9] = p + (q*s1 + r*c1)*t2;
     dxdt[10] = q*c1 - r*s1;
     dxdt[11] = (q*s1 + r*c1)*(1/cos(theta)); // replaced sec with 1/cos

     //dxdt[0] = F_r * cos(x[3]) + F_l * sin(x[3]);
     //dxdt[1] = F_r * sin(x[3]) + F_l * cos(x[3]);
     //dxdt[2] = r * F_r * sin(sigma) + r * F_l * cos(sigma);
     //
     //dxdt[3] = 20*x[0];
     //dxdt[4] = 20*x[1];
     //dxdt[5] = 20*x[2];
     //
     ////dxdt[0] = sigma * ( x[1] - x[0] );
     ////dxdt[1] = R * x[0] - x[1] - x[0] * x[2];
     ////dxdt[2] = -b * x[2] + x[0] * x[1];
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


void step_callback( const state_type &x , const double t )
{
     //cout << t << '\t' << x[0] << '\t' << x[1] << '\t' << x[2] << endl;
}

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

     ros::Time begin = ros::Time::now();
     ros::Time curr_time = begin;
     ros::Time prev_time = begin;
     ros::Duration dt = curr_time - prev_time;
     
     while (ros::ok())
     {
          curr_time = ros::Time::now();
          dt = curr_time - prev_time;
          prev_time = curr_time;

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
          //state_type x = {0,0,0,
          //                acceleration_.linear.x, acceleration_.linear.y,
          //                0};

          

          //stepper.do_step(videoray_model, x , t , dt ); 
          F_l = left_vel_;
          F_r = right_vel_;
          //stepper.do_step(videoray_model, x_ , curr_time.toSec() , dt.toSec()); 
          boost::numeric::odeint::integrate(videoray_model, 
                                            x_ , 
                                            curr_time.toSec() , 
                                            (curr_time + dt).toSec(), 
                                            dt.toSec(), 
                                            step_callback);


          //stepper.do_step(videoray_model, x_ , 0 , 1.0/10. );           
          //stepper.do_step(videoray_model, x , 0 , 1.0/10.0 );           
          
          ROS_INFO("========================");
          ROS_INFO("Current: %f, \tdt: %f", curr_time.toSec(), dt.toSec());
          ROS_INFO("Surge: %f", x_[0]);
          ROS_INFO("Sway: %f", x_[1]);
          ROS_INFO("Heave: %f", x_[2]);
          
          ROS_INFO("3: %f", x_[3]);
          ROS_INFO("4: %f", x_[4]);
          ROS_INFO("5: %f", x_[5]);
          ROS_INFO("6: %f", x_[6]);
          ROS_INFO("7: %f", x_[7]);
          ROS_INFO("8: %f", x_[8]);
          ROS_INFO("9: %f", x_[9]);
          ROS_INFO("10: %f", x_[10]);
          ROS_INFO("11: %f", x_[11]);
                    



          linear.x = x_[0];
          linear.y = x_[1];
          linear.z = x_[2];
          angular.z = x_[5];
          

          //linear.x = x[3];
          //linear.y = x[4];
          //angular.z = x[5];
          //
          //linear.z = vert_vel_;

          twist.linear = linear;
          twist.angular = angular;
          twist_pub.publish(twist);

          ros::spinOnce();

          loop_rate.sleep();
     }
     

     return 0;
}
