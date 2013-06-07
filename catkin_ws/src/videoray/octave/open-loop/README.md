Open Loop Simulation of VideoRay Pro III Dynamics
==================================================
Kevin DeMarco <demarco@gatech.edu>

Notes
-------
This simulation was written within the Octave framework.
http://www.gnu.org/software/octave/

Almost all of the code could easily be executed in Matlab, with just a
modification to the ode solver call.

Run the simulation
--------------------
1.) Start octave in this directory.
2.) Execute the sim_lsode.m script

Ubuntu example:
-------------------
$ cd /path/to/syllo-ros/catkin_ws/src/videoray/octave/open-loop
$ octave
$ sim_lsode

The sim_lsode call should generate six (6) figures that show the output
of the simulation.
