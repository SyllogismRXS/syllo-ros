#include "VideoRayComm.h"

#define PORT_THRUST_LSB  0
#define PORT_THRUST_MSB  1
#define STAR_THRUST_LSB  2
#define STAR_THRUST_MSB  3
#define VERT_THRUST_LSB  4
#define VERT_THRUST_MSB  5
#define LIGHTS_LSB       6
#define CAM_TILT         7
#define CAM_FOCUS        8
#define UKNOWN_0         9
#define UKNOWN_1         10
#define AUTO_DEPTH_LSB   11
#define AUTO_DEPTH_MSB   12
#define AUTO_HEADING_LSB 13
#define AUTO_HEADING_MSB 14

#define TX_CTRL_SIZE     15

using std::cout;
using std::endl;

VideoRayComm::VideoRayComm()
{
     packetizer_.set_network_id(0x01);
     packetizer_.set_flags(0x03);
     packetizer_.set_csr_addr(0x00);          

     int status;
     status = serial_.Open("/dev/ttyUSB0", 115200);
     if (status != 1) {
     	  cout << "Error while opening port. Permission problem ?" << endl;
     	  exit(-1);
     }
     serial_.FlushReceiver();
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

VideoRayComm::Status_t VideoRayComm::exec_transfer()
{
     char ** packet;
     int bytes;

     // Generate Packet and grab reference to it
     packetizer_.set_data(tx_ctrl_data, TX_CTRL_SIZE);
     bytes = packetizer_.generate_packet(packet);
     
     // Send the packet over the serial line
     serial_.Write((const void *)(*packet), bytes);

     char * byte;
     Packetizer::Status_t status;
     do {
          if (serial_.ReadChar(byte,0) == 1) {
               status = receiver_.receive_packet(*byte);
          } else {
               // Did not receive a byte, break out.
               break;
          }          
     } while(status == Packetizer::In_Progress);

     if (status == Packetizer::Success) {
          
     }

     return VideoRayComm::Success;
}

int VideoRayComm::heading()
{
     return heading_;
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

