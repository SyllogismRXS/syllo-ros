#ifndef LOGIC_H_
#define LOGIC_H_
/// ----------------------------------------------------------------------------
/// @file Logic.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-09-17 15:28:45 syllogismrxs>
///
/// @version 1.0
/// Created: 17 Sep 2013
///
/// ----------------------------------------------------------------------------
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
/// ----------------------------------------------------------------------------
/// @section DESCRIPTION
/// 
/// The Logic class ...
/// 
/// ----------------------------------------------------------------------------

#include <iostream>

#include <Stage-4.1/stage.hh>
#include "Robot.h"

using std::cout;
using std::endl;

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

#endif
