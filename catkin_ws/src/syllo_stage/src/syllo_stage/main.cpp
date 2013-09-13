#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sstream>
#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Stage-4.1/stage.hh>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include <FL/Fl.H>

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


class Logic
{
public:
     static int Callback(Stg::World* world, void* userarg)
          {
               Logic* lg = reinterpret_cast<Logic*>(userarg);
        
               lg->Tick(world);
        
               // never remove this call-back
               return 0;
          }
    
     Logic(unsigned int popsize)
          : 
          population_size(popsize),
          robots(new Robot[population_size])
          {

          }
    
     void connect(Stg::World* world, ros::NodeHandle * n)
          {
               // connect the first population_size robots to this controller
               for(unsigned int idx = 0; idx < population_size; idx++)
               {
                    // the robots' models are named r0 .. r1999
                    std::stringstream name;
                    name << "r" << idx;
                    
                    robots[idx].set_name(name.str());

                    // get the robot's model and subscribe to it
                    Stg::ModelPosition* posmod = reinterpret_cast<Stg::ModelPosition*>(
                         world->GetModel(name.str())
                         );
                    assert(posmod != 0);
            
                    robots[idx].position = posmod;
                    robots[idx].position->Subscribe();
            
                    // get the robot's ranger model and subscribe to it
                    Stg::ModelRanger* rngmod = reinterpret_cast<Stg::ModelRanger*>(
                         robots[idx].position->GetChild( "ranger:0" )
                         );
                    assert(rngmod != 0);
            
                    robots[idx].ranger = rngmod;
                    robots[idx].ranger->Subscribe();
                    
                    // Provide robot with ROS node handle
                    robots[idx].RegisterTopics(n);
                    

               }
        
               // register with the world
               world->AddUpdateCallback(Logic::Callback, reinterpret_cast<void*>(this));    
          }
    
     ~Logic()
          {
               delete[] robots;
          }
    

     void Tick(Stg::World*)
          {
               // the controllers parameters
               const double vspeed = 0.4; // meters per second
               const double wgain  = 1.0; // turn speed gain
               const double safe_dist = 0.1; // meters
               const double safe_angle = 0.3; // radians
    
               // each robot has a group of ir sensors
               // each sensor takes one sample
               // the forward sensor is the middle sensor
               for(unsigned int idx = 0; idx < population_size; idx++)
               {
                    Stg::ModelRanger* rgr = robots[idx].ranger;

                    // compute the vector sum of the sonar ranges	      
                    double dx=0, dy=0;

                    // the range model has multiple sensors
                    typedef std::vector<Stg::ModelRanger::Sensor>::const_iterator sensor_iterator;
                    const std::vector<Stg::ModelRanger::Sensor> sensors = rgr->GetSensors();
            
                    for(sensor_iterator sensor = sensors.begin(); sensor != sensors.end(); sensor++)
                    {
                         // each sensor takes a single sample (as specified in the .world)
                         const double srange = (*sensor).ranges[0];
                         const double angle  = (*sensor).pose.a;
                
                         dx += srange * std::cos(angle);
                         dy += srange * std::sin(angle);
                    } 
            
                    if(dx == 0)
                         continue;
            
                    if(dy == 0)
                         continue;
            
                    // calculate the angle towards the farthest obstacle
                    const double resultant_angle = std::atan2( dy, dx );

                    // check whether the front is clear
                    const unsigned int forward_idx = sensors.size() / 2u - 1u;
            
                    const double forwardm_range = sensors[forward_idx - 1].ranges[0];
                    const double forward_range  = sensors[forward_idx + 0].ranges[0]; 
                    const double forwardp_range = sensors[forward_idx + 1].ranges[0]; 
            
                    bool front_clear = (
                         (forwardm_range > safe_dist / 5.0) &&
                         (forward_range > safe_dist) &&
                         (forwardp_range > safe_dist / 5.0) &&
                         (std::abs(resultant_angle) < safe_angle)
                         );
            
                    // turn the sensor input into movement commands

                    // move forwards if the front is clear
                    const double forward_speed = front_clear ? vspeed : 0.0;
                    // do not strafe
                    const double side_speed = 0.0;	   
            
                    // turn towards the farthest obstacle
                    const double turn_speed = wgain * resultant_angle;
            
                    // finally, relay the commands to the robot
                    robots[idx].position->SetSpeed( forward_speed, side_speed, turn_speed );
                    
                    // Publish robot pose to ROS
                    robots[idx].publish_odometry();

               }
          }
    
protected:
     unsigned int population_size;   
     Robot* robots;
};


int main(int argc, char **argv)
{
     // check and handle the argumets
     if( argc < 3 ) {
          puts( "Usage: rosrun syllo_stage syllo_stage <worldfile> <number of robots>" );
          exit(0);
     }
     
     ros::init(argc, argv, "syllo_stage");
     ros::NodeHandle n;
     ros::Rate loop_rate(10);
     
     // Population size
     const int popsize = atoi(argv[2]);
     
     // initialize libstage
     Stg::Init( &argc, &argv );
     
     // create the world
     Stg::WorldGui world(800, 700, "Stage Benchmark Program");
     world.Load( argv[1] );
     
     // create the logic and connect it to the world
     Logic logic(popsize);
     logic.connect(&world, &n);
     
     // Start the simulation (so we don't have to hit pause/start button)
     world.Start();

     // and then run the simulation
     while (ros::ok() && !world.TestQuit()) {
          //std_msgs::String msg;
          //std::stringstream ss;
          //ss << "hello world";
          //msg.data = ss.str();
          //chatter_pub.publish(msg);

          ros::spinOnce();

          // Only allow the use of the GUI for now...
          // Se world.cc lines 221-228 for example check for non-GUI
          Fl::wait(loop_rate.expectedCycleTime().toSec());
     }
     
     cout << endl;
     cout << "==================================" << endl;
     cout << "Simulation Complete." << endl;

     exit(0);
     return 0;
}
