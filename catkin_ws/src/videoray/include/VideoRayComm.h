#ifndef VIDEORAYCOMM_H_
#define VIDEORAYCOMM_H_
/// ----------------------------------------------------------------------------
/// @file VideoRayComm.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2013-08-13 14:46:21 syllogismrxs>
///
/// @version 1.0
/// Created: 13 Aug 2013
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
/// The VideoRayComm class ...
/// 
/// ----------------------------------------------------------------------------

#include "Packetizer.h"
#include "serialib.h"

class VideoRayComm {
private:
     int heading_;
     int roll_;
     int pitch_;
     int water_temperature_;
     int internal_temperature_;
     int water_ingress_;
     int yaw_accel_;
     int pitch_accel_;
     int roll_accel_;
     int surge_accel_;
     int sway_accel_;
     int heave_accel_;
     
     char tx_ctrl_data[15];
     char rx_ctrl_Data[15];

     char tx_sensor_data[11];
     char rx_sensor_data[11];

     Packetizer packetizer_;
     Packetizer receiver_;
     serialib serial_;     

protected:
public:
     
     enum Status_t
     {
          Success = 0,
          Failure
     };

     VideoRayComm();
     ~VideoRayComm();
     
     Status_t set_desired_heading(int heading);
     Status_t set_desired_depth(int depth);
     Status_t set_focus(int focus);
     Status_t set_lights(int lights);
     Status_t set_vertical_thruster(int thrust);
     Status_t set_port_thruster(int thrust);
     Status_t set_starboard_thruster(int thrust);

     Status_t exec_transfer();

     int heading();
     int roll();
     int pitch();
     int water_temperature();
     int internal_temperature();
     int water_ingress();

     int yaw_accel();
     int pitch_accel();
     int roll_accel();
     int surge_accel();
     int sway_accel();
     int heave_accel();

};

#endif
