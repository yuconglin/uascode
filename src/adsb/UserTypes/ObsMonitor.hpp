#pragma once
#include "SageDecoder.hpp"
//#include "MsgSender.hpp"
#include "common/UserTypes/MavlinkSender.hpp"
#include "Planner/UserStructs/obstacle3D.h"
//for socket
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
//other
#include <vector>
//ros
#include "ros/ros.h"

#define BUFSIZE 2048
//#define PORT 4000
#define PORT 14550

namespace UasCode{
class ObsMonitor{
 public:
   ObsMonitor();
   //bool UartInit(const char *_uartname,const int _baudrate);
   int PortSetUp();
   int BytesDecode();
   inline void SetMavlinkSend(bool _send){mavlink_send=_send;}
   inline bool GetMavlinkSend(){return mavlink_send;}
 private:
   //for decoding
   SageDecoder decoder;
   //for msg send to pixhawk
   bool mavlink_send;
   MavlinkSender sender;
   //for adsb record
   //for convert to mavlink msg
   std::vector<UserStructs::obstacle3D> obss;
   //ros related
   //ros::NodeHandle nh;
   //ros::Publisher pub_obss;
};// ObsMonitor ends
}//namespace ends
