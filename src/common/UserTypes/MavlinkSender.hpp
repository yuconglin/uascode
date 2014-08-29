#pragma once
#include <netdb.h>
#include "stdio.h"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>

#include "Planner/UserStructs/obstacle3D.h"
#include <vector>

namespace UasCode{

class MavlinkSender{
 public:
  MavlinkSender();
  ~MavlinkSender();

  int initialize();
  //sending
  void SendPosSP(double lat,double lon,double alt);//position setpoint
  void SendPosSPflag(double lat,double lon,double alt,int index,int inter_exist);
  void SendIfColli(bool if_colli);
  void SendMultiObs(std::vector<UserStructs::obstacle3D> obss);
  void SendWpNum(int wp_num);
  void SendMultiObs3(std::vector<UserStructs::obstacle3D> obss);
  void SendColliPt(double lat_c,double lon_c,double alt_c);
private:
  struct addrinfo *p; 
  int sockfd;
  //struct sockaddr_in ip4addr;
};

};
