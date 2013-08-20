#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "VideoRayComm.h"
#include "Joystick.h"

#include <sstream>

using std::cout;
using std::endl;

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

double invert_sign(double input)
{
     return -1*input;
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

int main(int argc, char **argv)
{
     std::ofstream file;
     file.open("data.log", std::ios::app);
     file << "------------------------" << endl;     

     ros::init(argc, argv, "talker");
     ros::NodeHandle n;

     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

     ros::Rate loop_rate(100);

     //VideoRayComm::Status_t status;
     //status = VideoRayComm::failure;
     VideoRayComm::Status_t status;

     //if (status == VideoRayComm::Success) {
     //}

     VideoRayComm comm;
     
     Joystick js_;
     int *axis_;
     int *button_;
     int num_of_buttons_;
     int num_of_axis_;

     js_.init("/dev/input/js0");
     num_of_axis_ = js_.num_of_axis();
     num_of_buttons_ = js_.num_of_buttons();     

     int vert_thrust_ = 0;
     int port_thrust_ = 0;
     int star_thrust_ = 0;
     int turn_ = 0;
     int lights_ = 0;
     int tilt_ = 0;
     int focus_ = 0;

     bool started_ = false;
     bool logging_ = false;
     
     int count = 0;
     while (ros::ok()) {
          
          js_.update(&axis_, &button_);
     
          //cout << "Vertical: " << axis_[1] << endl;
          vert_thrust_ = invert_sign(normalize(axis_[1], -32767, 32767, -99, 99));
          port_thrust_ = invert_sign(normalize(axis_[3], -32767, 32767, -89, 89));
          star_thrust_ = invert_sign(normalize(axis_[3], -32767, 32767, -89, 89));

          turn_ = invert_sign(normalize(axis_[2], -32767, 32767, -89, 89));
          port_thrust_ -= turn_;
          star_thrust_ += turn_;
          
          //cout << axis_[0] << axis_[1] << axis_[2] << axis_[3] << axis_[4] << axis_[5] << endl;

          tilt_ += invert_sign(normalize(axis_[5], -32767, 32767, -1, 1));
          tilt_ = saturate(tilt_, -80, 80);

          focus_ += invert_sign(normalize(axis_[4], -32767, 32767, -1, 1));
          focus_ = saturate(tilt_, -100, 100);

          if (button_[3]) {
               lights_ += 1;
          } else if (button_[1]) {
               lights_ -= 1;
          }
          lights_ = saturate(lights_, 0, 63);

          if (button_[8]) {
               logging_ = !logging_;
          }

          // Reset button
          if (button_[9]) {
               logging_ = false;
               started_ = true;
               tilt_ = 0;
               focus_ = 0;
               lights_ = 0;
               port_thrust_ = 0;
               star_thrust_ = 0;
               vert_thrust_ = 0;               
          }

          status = comm.set_vertical_thruster(vert_thrust_);
          status = comm.set_port_thruster(port_thrust_);
          status = comm.set_starboard_thruster(star_thrust_);
          status = comm.set_lights(lights_);
          status = comm.set_camera_tilt(tilt_);
          
          if (started_) {
               status = comm.send_control_command();
               if (status != VideoRayComm::Success) {
                    cout << "Exec Transfer Error!" << endl;
               }
               
               if (logging_) {
                    status = comm.send_sensor_command();
                    if (status != VideoRayComm::Success) {
                         cout << "Exec Transfer Error!" << endl;
                    }
                    //cout << "------------------------------------" << endl;
                    //cout << "Heading: " << comm.heading() << endl;
                    //cout << "Pitch: " << comm.pitch() << endl;
                    //cout << "Roll: " << comm.roll() << endl;
                    //cout << "Depth: " << comm.depth() << endl;
                    //cout << "Yaw Accel: " << comm.yaw_accel() << endl;
                    //cout << "Pitch Accel: " << comm.pitch_accel() << endl;
                    //cout << "Roll Accel: " << comm.roll_accel() << endl;
                    //cout << "Surge Accel: " << comm.surge_accel() << endl;
                    //cout << "Sway Accel: " << comm.sway_accel() << endl;
                    //cout << "Heave Accel: " << comm.heave_accel() << endl;
                    
                    file << comm.heading() << endl;

               }
          }

          
          //cout << "Count: " << count << endl;
          //status = comm.set_desired_heading(count);
          //if (status != VideoRayComm::Success) {
          //     cout << "Error detected!" << endl;
          //}
          
          //std_msgs::String msg;
          //
          //std::stringstream ss;
          //ss << "hello world " << count;
          //msg.data = ss.str();
          //
          //ROS_INFO("%s", msg.data.c_str());
          //
          //chatter_pub.publish(msg);

          ros::spinOnce();

          loop_rate.sleep();
          ++count;
     }
     
     cout << "Closing log file" << endl;
     file.close();

     return 0;
}
