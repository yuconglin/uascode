#pragma once
#include "UserStructs/AdsbMsg.h"
#include "UserStructs/AdsbQGC.h"
//std
#include <vector>

namespace UasCode{

class MsgSender{
 public:
  //constructor
  MsgSender();
  MsgSender(char *_uartname,int _baudrate);
  ~MsgSender();
  //other normal functions
  int OpenPort();//return -1 if open fails otherwise return the port number; using open_port(char* port) function
  bool SetupPort();
  void ClosePort();
  //for sending messages
  int SendAdsbMsg(const UserStructs::AdsbMsg adsb_m);
  int SendMultiAdsbs(const std::vector<UserStructs::AdsbQGC> qgc_adsbs);

  void SetUartname(const char *_uartname);
  void inline SetBaudrate(const int _baudrate){baudrate= _baudrate;}
 private:
  int fd; //file descriptor
  char *uart_name;//pay attention to assign
  int baudrate;

};//class ends

};//namespace ends
