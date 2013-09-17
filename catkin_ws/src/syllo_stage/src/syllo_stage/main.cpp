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

#include "Logic.h"

using std::cout;
using std::endl;

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
