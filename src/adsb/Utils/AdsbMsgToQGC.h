#pragma once
#include "adsb/UserStructs/AdsbMsg.h"
#include "adsb/UserStructs/AdsbQGC.h"

namespace Utils{

  UserStructs::AdsbQGC AdsbMsgToQGC(UserStructs::AdsbMsg msg);

};//namespace Utils ends

