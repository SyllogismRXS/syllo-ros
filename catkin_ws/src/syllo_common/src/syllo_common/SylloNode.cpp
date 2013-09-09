#include <iostream>
#include "ros/ros.h"
#include "syllo_common/SylloNode.h"

using std::cout;
using std::endl;

SylloNode::SylloNode()
{
}

int SylloNode::init()
{

     ros::param::param<int>("tick_rate", tick_rate_, 100);     
     ros_tick_rate_ = new ros::Rate(tick_rate_);
     
     std::cout << "================================" << endl;
     std::cout << "               _ _         " << std::endl; 
     std::cout << "     ___ _   _| | | ___    " << std::endl;
     std::cout << "    / __| | | | | |/ _ \\  " << std::endl;
     std::cout << "    \\__ \\ |_| | | | (_) |" << std::endl;
     std::cout << "    |___/\\__, |_|_|\\___/ " << std::endl; 
     std::cout << "         |___/             " << std::endl;
     std::cout << "================================" << endl;
     std::cout << "Tick Rate: " << tick_rate_ << std::endl;              

     return 0;
}

int SylloNode::spin()
{
     ros::spinOnce();
     ros_tick_rate_->sleep();
     
     return 0;
}

int SylloNode::cleanup()
{
     if (ros_tick_rate_) {
          delete ros_tick_rate_;
     }
     return 0;
}
