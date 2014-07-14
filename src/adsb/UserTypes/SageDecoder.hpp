#pragma once
//header file for Sagetech ads-b receiver decoding
//#include "UserStructs/AdsbMsg.h"
//#include "UserStructs/HeartMsg.h"
#include "UserStructs/DecodeMsg.h"
//#include <vector>

namespace UasCode{

class SageDecoder{
  public:
    //the general function to decode three needed message
    int Decode(unsigned char *buf, int len);
    inline UserStructs::AdsbMsg GetAdsb() {return this->sage_adsb;}
    inline UserStructs::HeartMsg GetHeart() {return this->heartbeat;}
    inline UserStructs::StatusMsg GetStatus() {return this->sage_status;}
    inline UserStructs::OwnerMsg GetOwnership() {return ownership;}
  private:
    //heartbeat is needed for time stamp
    //empty temperarily

    //device status
    //empty temperarily

    //GPS NMEA
    //empty temperarily

    //msgs
    UserStructs::AdsbMsg sage_adsb;
    UserStructs::HeartMsg heartbeat; 
    UserStructs::StatusMsg sage_status;
    UserStructs::OwnerMsg ownership;
};

}//namespace UasCode ends
