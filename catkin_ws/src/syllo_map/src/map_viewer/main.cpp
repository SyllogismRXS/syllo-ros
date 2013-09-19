/// main.cpp  ----------------------------------------------------------------
/// @file main.cpp
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-09-18 21:49:58 syllogismrxs>
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

#include "boost/multi_array.hpp"
#include <cassert>

using std::endl;
using std::cout;

cv::Mat cv_map_;
nav_msgs::MapMetaData info_;

typedef boost::multi_array<int, 2> array_type;
typedef array_type::index idx;
array_type map_;


void callback_map(const nav_msgs::OccupancyGridConstPtr& msg)
{
     info_ = msg->info;
    
     int height = info_.height;
     int width = info_.width;

     map_.resize(boost::extents[width][height]);
     for (int x = 0; x < width; x++) {
          for (int y = 0; y < height; y++) {
               map_[x][y] = msg->data[y*width + x];
          }
     }

     cv_map_ = cv::Mat(height, width, CV_8UC1);     
     for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {
               double value = map_[x][y];
               value = normalize(value, 0, 100, 0, 255);
               cv_map_.at<uchar>(height-1-y,x) = 255 - value;
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

          if (!cv_map_.empty()) {
               cv::imshow("Map", cv_map_);
               cv::waitKey(1);
          }

          ros::spinOnce();
          loop_rate.sleep();
     }

     return 0;
}
