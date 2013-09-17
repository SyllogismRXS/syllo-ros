/// main.cpp  ----------------------------------------------------------------
/// @file main.cpp
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-09-17 17:42:43 syllogismrxs>
///
/// @version 1.0
/// Created: 17 Sep 2013
///
/// ---------------------------------------------------------------------------
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
/// ---------------------------------------------------------------------------
/// @section DESCRIPTION
/// 
/// ---------------------------------------------------------------------------

#include <iostream>

// ROS Headers
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

// OpenCV headers
#include <cv.h>
#include <highgui.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <syllo_common/Filter.h>

using std::endl;
using std::cout;

cv::Mat map_;
nav_msgs::MapMetaData info_;

void callback_map(const nav_msgs::OccupancyGridConstPtr& msg)
{
     info_ = msg->info;
     
     map_ = cv::Mat(info_.height, info_.width, CV_8UC1);
     
     for (int r = 0; r < info_.height; r++) {
          for (int c = 0; c < info_.width; c++) {
               double value = msg->data[r*info_.width+c];
               value = normalize(value, 0, 100, 0, 255);
               map_.at<uchar>(info_.height-1-r,c) = 255 - value;
          }
     }

}

int main(int argc, char * argv[])
{
     ros::init(argc, argv, "syllo_stage");
     ros::NodeHandle n;
     ros::Rate loop_rate(5);

     ros::Subscriber map_sub;
     map_sub = n.subscribe("/map", 1, callback_map);

     while (ros::ok()) {

          if (!map_.empty()) {
               cv::imshow("Map", map_);
               cv::waitKey(1);
          }

          ros::spinOnce();
          loop_rate.sleep();
     }

     return 0;
}
