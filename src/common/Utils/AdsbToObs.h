#pragma once
#include "adsb/UserStructs/AdsbMsg.h"
#include "Planner/UserStructs/obstacle3D.h"
#include "yucong_rosmsg/ObsMsg2.h"

namespace Utils{

  void AdsbToObs(const UserStructs::AdsbMsg &adsb,
                 UserStructs::obstacle3D &obs3D);

  void AdsbToObsMsg2(const UserStructs::AdsbMsg &adsb,yucong_rosmsg::ObsMsg2 msg2);

}
