#!/bin/bash

catkin_make clean

rm -rf build
rm -rf devel
rm -rf install

rm -rf ./src/videoray/gui/*build*
rm -rf ./src/videoray/gui/videoray/*.user
