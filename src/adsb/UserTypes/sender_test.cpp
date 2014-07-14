#include "MsgSender.hpp"
#include <unistd.h>
using namespace UasCode;

int main(int argc,char** argv)
{
  MsgSender sender("/dev/ttyACM0",115200);
  sender.OpenPort();
  sender.SetupPort();
  //send a adsb message
  UserStructs::AdsbMsg adsb_m;
  //alert:0 address_type:0 address:11013548 33.5376 -111.61 19000 Mi:9 NIC:8 NACp:8 v:416 vv:2176 hd:61.875 0 NKS168  0 3
  adsb_m.address= 11013548;
  adsb_m.latitude= 33.5376;
  adsb_m.longitude= -111.61;
  adsb_m.altitude= 19000;
  adsb_m.hd= 61.875;
  //sending
  for(int i=0;i<20;++i){
    sender.SendAdsbMsg(adsb_m);
    sleep(1);
  }
  return 0;
}
