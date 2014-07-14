#pragma once
#include <netdb.h>
#include "stdio.h"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>

namespace UasCode{

class MavlinkSender{
 public:
  //MavlinkSender();
  //~MavlinkSender();
  //
  int initialize();
  //sending
  void SendPosSP(double lat,double lon,double alt);//position setpoint
 private:
  struct addrinfo *p; 
  int sockfd;
  //struct sockaddr_in ip4addr;
};

};
