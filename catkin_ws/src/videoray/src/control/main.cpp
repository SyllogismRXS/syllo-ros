#include "ros/ros.h"
#include "std_msgs/String.h"
#include "VideoRayComm.h"

#include <sstream>

using std::cout;
using std::endl;

int main(int argc, char **argv)
{
     ros::init(argc, argv, "talker");
     ros::NodeHandle n;

     ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

     ros::Rate loop_rate(10);

     //VideoRayComm::Status_t status;
     //status = VideoRayComm::failure;
     VideoRayComm::Status_t status;

     //if (status == VideoRayComm::Success) {
     //}

     VideoRayComm comm;
     

     int count = 0;
     while (ros::ok()) {
          
          cout << "Count: " << count << endl;
          status = comm.set_desired_heading(count);
          if (status != VideoRayComm::Success) {
               cout << "Error detected!" << endl;
          }
                    

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
     return 0;
}
