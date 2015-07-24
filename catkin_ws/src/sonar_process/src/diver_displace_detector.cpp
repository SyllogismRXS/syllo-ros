#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv_workbench/detector/Detector.h>
#include <opencv_workbench/plugin_manager/PluginManager.h>

using std::cout;
using std::endl;

PluginManager<Detector, Detector_maker_t> plugin_manager_;
Detector * detector_;

int frame_number_ = 0;

void sonarCallback(const sensor_msgs::ImageConstPtr &msg)
{
     cv_bridge::CvImagePtr cv_img_ptr = cv_bridge::toCvCopy(msg, "bgr8");      
     cv::Mat original = cv_img_ptr->image;

     // Pass the frame to the detector plugin
     detector_->set_frame(frame_number_, original);

     // Get track list from detector
     std::vector<syllo::Track> tracks = detector_->tracks();
     
     // Draw estimated diver locations on original image
     std::vector<syllo::Track>::iterator it = tracks.begin();
     for (; it != tracks.end(); it++) {
          cv::Point3d point3d = it->position();
          
          if (it->type() == syllo::Diver) {
               // If this is a diver type, mark it on the original image                    
               int radius = 3;
               cv::circle(original, cv::Point(point3d.x,point3d.y), 
                          radius, cv::Scalar(0,0,0), 2, 8, 0);
          }
     }                    
     cv::imshow("Detection", original);
     cv::waitKey(1);
     frame_number_++;
}

int main(int argc, char **argv)
{    
     std::string plugin_name = "displace_detector";

     // Load the Bridge shared library (based on yml file)
     int retcode = plugin_manager_.search_for_plugins("OPENCV_WORKBENCH_PLUGIN_PATH");
     if (retcode != 0) {
          cout << "Failed to find plugins." << endl;
          return -1;
     }

     retcode = plugin_manager_.open_library(plugin_name);
     if (retcode != 0) {
          cout << "Unable to open library: " << plugin_name << endl;
          
          return -1;
     } else {
          cout << "Using Bridge Library: " << plugin_name << endl;
     }
     
     detector_ = plugin_manager_.object();
     detector_->print();       
     detector_->hide_windows(false);     

     ros::init(argc, argv, "diver_displace_detector");
     ros::NodeHandle nh_;
     
     //Subscribe to range commands
     ros::Subscriber sonar_sub = nh_.subscribe("/sonar_xy_0", 1, sonarCallback);     

     ros::Rate loop_rate(10);
     while (ros::ok()) {          
          ros::spinOnce(); 
          loop_rate.sleep();
     }     
     return 0;
}
