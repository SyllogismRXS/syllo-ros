Installation Notes
================================================================================
These ROS packages were developed and tested on Ubuntu. 

Contact Kevin DeMarco <kevin.demarco@gmail.com> if you have difficulties 
installing or using any of the packages.

Commands that should be entered at the terminal are denoted with a '$'

Dependencies
------------
ROS - www.ros.org  

Acquiring the Code
------------------
Install git  
$ sudo apt-get install git-core

I keep all of my git repositories in the same folder for organizational 
reasons.  Typically, I keep them in the ~/git-repos folder.  So create it...

$ mkdir ~/git-repos  
$ cd ~/git-repos

Make a clone of the repo  
$ git clone git://github.com/SyllogismRXS/syllo-ros.git

Building the Code
-----------------
Add your new repo to your $ROS_PACKAGE_PATH environment variable.  
$ echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/git-repos/moos-ros-bridge" >> ~/.bashrc  
$ . ~/.bashrc

Of course, if you didn't clone your repo into ~/git-repos/, then you will need 
to modify the previous command accordingly.

Reindex your ROS packages  
$ rospack profile

Build moosros_tester  
$ rosmake <package>

Launch Example
--------------
TODO

----------------------------------------
Kevin DeMarco <kevin.demarco@gmail.com>
