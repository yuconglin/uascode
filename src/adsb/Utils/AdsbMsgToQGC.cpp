#include "AdsbMsgToQGC.h"

namespace Utils{

 UserStructs::AdsbQGC AdsbMsgToQGC(UserStructs::AdsbMsg msg)
 {
   UserStructs::AdsbQGC adsb_qgc;
   adsb_qgc.address= msg.address;
   adsb_qgc.latitude= msg.latitude;
   adsb_qgc.longitude= msg.longitude;
   adsb_qgc.altitude= msg.altitude;
   adsb_qgc.hd= msg.hd;
   return adsb_qgc;
 }

};
