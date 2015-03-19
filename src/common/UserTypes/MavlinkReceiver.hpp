#pragma once
//for ros
#include "ros/ros.h"
#include "yc_common/mavlink.h"

namespace UasCode{

class MavlinkReceiver{
 public:
   //constructor
   MavlinkReceiver();
   MavlinkReceiver(const char* _uartname,int _baudrate);
   ~MavlinkReceiver();
   //other functions
   int OpenPort();//return -1 if open fails
   bool SetupPort();
   void ClosePort();
   //for continously receive messages
   int ReceiveMsg();
   //trival functions
   void SetUartname(const char* _uartname);
   void inline SetBaudrate(const int _baudrate){baudrate= _baudrate;}
   void inline SetIfHil(const bool _if){if_hil= _if;}

private:
   int fd;//file descriptor
   char *uart_name;
   int baudrate;
   bool if_hil;
   //ros related
   ros::NodeHandle nh;
   //publisher
   ros::Publisher pub_pos;
   ros::Publisher pub_att;
   ros::Publisher pub_goal;
   //subscriber
   ros::Subscriber sub_interwp;
   //functions to handle messages
   void handle_message(mavlink_message_t *msg);
   void handle_message_gps_raw(mavlink_message_t *msg);
   void handle_message_attitude(mavlink_message_t *msg);   
   void handle_message_global_pos(mavlink_message_t *msg);
   void handle_message_yc_hil_attitude(mavlink_message_t *msg); 
   void handle_message_global_pos_setpoint(mavlink_message_t *msg);
};//class ends


};//namespace ends
