#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>
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

class ROSHandler
{
public:
     ROSHandler() {
          frame_number_ = 0;                    
     }

     void setup_topics()
          {
               //sonar_sub_ = n_.subscribe("/sonar_xy_0", 1, &ROSHandler::sonarCallback, this); 
               sonar_sub_ = n_.subscribe("/sonar_image", 1, &ROSHandler::sonarCallback, this); 
          }

     bool load_plugin(std::string plugin_name)
          {
               // Load the Bridge shared library (based on yml file)
               int retcode = plugin_manager_.search_for_plugins("OPENCV_WORKBENCH_PLUGIN_PATH");
               if (retcode != 0) {
                    cout << "Failed to find plugins." << endl;               
               }

               retcode = plugin_manager_.open_library(plugin_name);
               if (retcode != 0) {
                    cout << "Unable to open library: " << plugin_name << endl;
               } else {
                    cout << "Using Bridge Library: " << plugin_name << endl;
               }
     
               detector_ = plugin_manager_.object();
               detector_->print();       
               detector_->hide_windows(false);     
          }
     
     void sonarCallback(const sensor_msgs::ImageConstPtr &msg)
          {
               //cv::Mat original = cv::imdecode(cv::Mat(msg));
               cv_bridge::CvImagePtr cv_img_ptr = cv_bridge::toCvCopy(msg, "bgr8");      
               cv::Mat original = cv_img_ptr->image;
     
               // Pass the frame to the detector plugin
               detector_->set_frame(frame_number_, original);
     
               // Get track list from detector
               std::vector<wb::Entity> tracks = detector_->tracks();
     
               // Draw estimated diver locations on original image
               std::vector<wb::Entity>::iterator it = tracks.begin();
               for (; it != tracks.end(); it++) {
                    cv::Point centroid = it->centroid();
                    
                    if (it->type() == wb::Entity::Diver) {
                         // If this is a diver type, mark it on the original image                    
                         int radius = 3;
                         cv::circle(original, centroid, 
                                    radius, cv::Scalar(0,0,0), 2, 8, 0);
                    }
               }          
     
               cv::imshow("Detection", original);
               cv::waitKey(1);
               frame_number_++;
          }
          
protected:     
     Detector * detector_;
     
     int frame_number_;

     ros::NodeHandle n_; 
     ros::Publisher pub_;
     
     ros::Subscriber sonar_sub_;

     geometry_msgs::PoseStamped pose_;

private:
};

int main(int argc, char **argv)
{    
     ros::init(argc, argv, "diver_displace_detector");          
     
     if (argc < 2) {
          cout << "============================================" << endl;
          cout << "Usage: " << argv[0] << " <plugin_library>" << endl;
          return -1;
     }

     std::string plugin_library = std::string(argv[1]);
     
     ROSHandler ros_handler;

     ros_handler.load_plugin(plugin_library);
     ros_handler.setup_topics();
     
     ros::Rate loop_rate(10);
     while (ros::ok()) {          
          ros::spinOnce(); 
          loop_rate.sleep();
     }     
     return 0;
}
