#include <iostream>
#include <stdio.h>

#include "VideoRayComm.h"

//////////////////////////////
// TX Control Packet Defines
//////////////////////////////
#define PORT_THRUST_LSB  0
#define PORT_THRUST_MSB  1
#define STAR_THRUST_LSB  2
#define STAR_THRUST_MSB  3
#define VERT_THRUST_LSB  4
#define VERT_THRUST_MSB  5
#define LIGHTS_LSB       6
#define CAM_TILT         7
#define CAM_FOCUS        8
#define UNKNOWN_0        9
#define UNKNOWN_1        10
#define AUTO_DEPTH_LSB   11
#define AUTO_DEPTH_MSB   12
#define AUTO_HEADING_LSB 13
#define AUTO_HEADING_MSB 14

#define TX_CTRL_SIZE     15

////////////////////////////////
//// RX Control Packet Defines
////////////////////////////////
//#define UNKNOWN_2   0
//#define HEADING_LSB 1
//#define HEADING_MSB 2
//#define PITCH_LSB   3
//#define PITCH_MSB   4
//#define ROLL_LSB    5
//#define ROLL_MSB    6

//////////////////////////////
// RX Control Packet Defines
//////////////////////////////
#define DEVICE_ID      0
#define DEPTH_LSB      1
#define DEPTH_MSB      2
#define HEADING_LSB    3
#define HEADING_MSB    4
#define PITCH_LSB      5
#define PITCH_MSB      6
#define ROLL_LSB       7
#define ROLL_MSB       8
#define YAW_ACC_LSB    9
#define YAW_ACC_MSB    10
#define PITCH_ACC_LSB  11
#define PITCH_ACC_MSB  12
#define ROLL_ACC_LSB   13
#define ROLL_ACC_MSB   14
#define SURGE_ACC_LSB  15
#define SURGE_ACC_MSB  16
#define SWAY_ACC_LSB   17
#define SWAY_ACC_MSB   18
#define HEAVE_ACC_LSB  19
#define HEAVE_ACC_MSB  20

using std::cout;
using std::endl;

VideoRayComm::VideoRayComm()
{
     packetizer_.set_network_id(0x01);
     
     int status;
     status = serial_.Open("/dev/ttyUSB0", 115200);
     if (status != 1) {
     	  cout << "Error while opening port. Permission problem ?" << endl;
     	  exit(-1);
     }
     serial_.FlushReceiver();

     tx_ctrl_data[PORT_THRUST_LSB] = 0;
     tx_ctrl_data[PORT_THRUST_MSB] = 0;
     tx_ctrl_data[STAR_THRUST_LSB] = 0;
     tx_ctrl_data[STAR_THRUST_MSB] = 0;
     tx_ctrl_data[VERT_THRUST_LSB] = 0;
     tx_ctrl_data[VERT_THRUST_MSB] = 0;
     tx_ctrl_data[LIGHTS_LSB] = 0;
     tx_ctrl_data[CAM_TILT] = 0;
     tx_ctrl_data[CAM_FOCUS] = 0;
     tx_ctrl_data[UNKNOWN_0] = 0;
     tx_ctrl_data[UNKNOWN_1] = 0;
     tx_ctrl_data[AUTO_DEPTH_LSB] = 0xFF;
     tx_ctrl_data[AUTO_DEPTH_MSB] = 0xFF;
     tx_ctrl_data[AUTO_HEADING_LSB] = 0xFF;
     tx_ctrl_data[AUTO_HEADING_MSB] = 0xFF;
     
     heading_ = 0;
     pitch_ = 0;
     roll_ = 0;
}

VideoRayComm::~VideoRayComm()
{
     serial_.Close();
}

VideoRayComm::Status_t VideoRayComm::set_desired_heading(int heading)
{
     if (heading > 360 || heading <= 0) {
          tx_ctrl_data[AUTO_HEADING_LSB] = 0xFF;
          tx_ctrl_data[AUTO_HEADING_MSB] = 0xFF;
     } else {
          tx_ctrl_data[AUTO_HEADING_LSB] = heading & 0x00FF;
          tx_ctrl_data[AUTO_HEADING_MSB] = (heading & 0xFF00) >> 8;
     }

     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_desired_depth(int depth)
{
     if (depth < 0) {
          tx_ctrl_data[AUTO_DEPTH_LSB] = 0xFF;
          tx_ctrl_data[AUTO_DEPTH_MSB] = 0xFF;
     } else {
          tx_ctrl_data[AUTO_DEPTH_LSB] = depth & 0x00FF;
          tx_ctrl_data[AUTO_DEPTH_MSB] = (depth & 0xFF00) >> 8;
     }
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_focus(int focus)
{
     tx_ctrl_data[CAM_FOCUS] = focus;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_camera_tilt(int tilt)
{
     tx_ctrl_data[CAM_TILT] = tilt;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_lights(int lights)
{
     tx_ctrl_data[LIGHTS_LSB] = lights;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_vertical_thruster(int thrust)
{
     tx_ctrl_data[VERT_THRUST_LSB] = thrust & 0x00FF;
     tx_ctrl_data[VERT_THRUST_MSB] = (thrust & 0xFF00) >> 8;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_port_thruster(int thrust)
{
     tx_ctrl_data[PORT_THRUST_LSB] = thrust & 0x00FF;
     tx_ctrl_data[PORT_THRUST_MSB] = (thrust & 0xFF00) >> 8;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::set_starboard_thruster(int thrust)
{
     tx_ctrl_data[STAR_THRUST_LSB] = thrust & 0x00FF;
     tx_ctrl_data[STAR_THRUST_MSB] = (thrust & 0xFF00) >> 8;
     return VideoRayComm::Success;
}

VideoRayComm::Status_t VideoRayComm::send_control_command()
{
     char * packet;
     int bytes;

     // Generate Packet and grab reference to it
     packetizer_.set_flags(0x03);
     packetizer_.set_csr_addr(0x00);          
     packetizer_.set_data(tx_ctrl_data, TX_CTRL_SIZE);
     bytes = packetizer_.generate_packet(&packet);    
     
     // Send the Tx Control packet over the serial line
     serial_.Write((const void *)packet, bytes);
     
     char byte;
     Packetizer::Status_t status;
     do {
          if (serial_.ReadChar(&byte,0) == 1) {
               status = receiver_.receive_packet(byte);
          } else {
               // Did not receive a byte, break out.
               printf("Error reading byte.\n");
               break;
          }          
     } while(status == Packetizer::In_Progress);
     
     if (status == Packetizer::Success) {
          bytes = receiver_.get_payload(&packet);
          
          //for (int x = 0 ; x < bytes ; x++) {
          //     printf("%x ", (unsigned char)packet[x]);
          //}
          //printf("\n");

          //short temp = 0;
          //temp = ((short)(packet[HEADING_MSB]) << 8) | packet[HEADING_LSB]; 
          //heading_ = temp;
          //
          //temp = 0;
          //temp = ((short)(packet[PITCH_MSB]) << 8) | packet[PITCH_LSB]; 
          //pitch_ = temp;
          //
          //temp = 0;
          //temp = ((short)(packet[ROLL_MSB]) << 8) | packet[ROLL_LSB]; 
          //roll_ = temp;
                    
     } else {
          printf("Decode Error.\n");
     }
     return VideoRayComm::Success;
}


VideoRayComm::Status_t VideoRayComm::send_sensor_command()
{
     char * packet;
     int bytes;

     //////////////////////
     // Tx Sensor Message
     //////////////////////
     // Generate Packet and grab reference to it
     //packetizer_.set_flags(0x8C);
     //packetizer_.set_flags(0xA0);
     //packetizer_.set_csr_addr(0x6E);
     packetizer_.set_flags(0x94);
     packetizer_.set_csr_addr(0x66);          
     packetizer_.set_data(tx_ctrl_data, 0);
     bytes = packetizer_.generate_packet(&packet);
     
     //for (int x = 0 ; x < bytes ; x++) {
     //     printf("%x ", (unsigned char)packet[x]);
     //}
     //printf("\n");
     
     // Send the Tx Control packet over the serial line
     serial_.Write((const void *)packet, bytes);
     
     char byte;
     Packetizer::Status_t status;
     do {
          if (serial_.ReadChar(&byte,0) == 1) {
               status = receiver_.receive_packet(byte);
          } else {
               // Did not receive a byte, break out.
               printf("Error reading byte.\n");
               break;
          }          
          //printf("%x ", (unsigned char)byte);
     } while(status == Packetizer::In_Progress);
     //printf("\n");

     if (status == Packetizer::Success) {
          bytes = receiver_.get_payload(&packet);
          
          //for (int x = 0 ; x < bytes ; x++) {
          //     printf("%x ", (unsigned char)packet[x]);
          //}
          //printf("\n");
          
          short temp = 0;
          temp = ((short)(packet[DEPTH_MSB]) << 8) | packet[DEPTH_LSB]; 
          depth_ = temp;
                    
          temp = ((short)(packet[HEADING_MSB]) << 8) | packet[HEADING_LSB]; 
          heading_ = temp;
          
          temp = 0;
          temp = ((short)(packet[PITCH_MSB]) << 8) | packet[PITCH_LSB]; 
          pitch_ = temp;
          
          temp = 0;
          temp = ((short)(packet[ROLL_MSB]) << 8) | packet[ROLL_LSB]; 
          roll_ = temp;

          temp = 0;
          temp = ((short)(packet[YAW_ACC_MSB]) << 8) | packet[YAW_ACC_LSB]; 
          yaw_accel_ = temp;
          
          temp = 0;
          temp = ((short)(packet[PITCH_ACC_MSB]) << 8) | packet[PITCH_ACC_LSB]; 
          pitch_accel_ = temp;
          
          temp = 0;
          temp = ((short)(packet[ROLL_ACC_MSB]) << 8) | packet[ROLL_ACC_LSB]; 
          roll_accel_ = temp;
          
          temp = 0;
          temp = ((short)(packet[SURGE_ACC_MSB]) << 8) | packet[SURGE_ACC_LSB]; 
          surge_accel_ = temp;
          
          temp = 0;
          temp = ((short)(packet[SWAY_ACC_MSB]) << 8) | packet[SWAY_ACC_LSB]; 
          sway_accel_ = temp;
          
          temp = 0;
          temp = ((short)(packet[HEAVE_ACC_MSB]) << 8) | packet[HEAVE_ACC_LSB]; 
          heave_accel_ = temp;
                    
     } else {
          printf("Decode Error.\n");
     }
     return VideoRayComm::Success;
}

int VideoRayComm::heading()
{
     return heading_;
}

int VideoRayComm::depth()
{
     return depth_;
}


int VideoRayComm::roll()
{
     return roll_;
}

int VideoRayComm::pitch()
{
     return pitch_;
}

int VideoRayComm::water_temperature()
{
     return water_temperature_;
}

int VideoRayComm::internal_temperature()
{
     return internal_temperature_;
}

int VideoRayComm::water_ingress()
{
     return water_ingress_;
}

int VideoRayComm::yaw_accel()
{
     return yaw_accel_;
}

int VideoRayComm::pitch_accel()
{
     return pitch_accel_;
}

int VideoRayComm::roll_accel()
{
     return roll_accel_;
}

int VideoRayComm::surge_accel()
{
     return surge_accel_;
}

int VideoRayComm::sway_accel()
{
     return sway_accel_;
}

int VideoRayComm::heave_accel()
{
     return heave_accel_;
}

