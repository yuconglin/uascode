#pragma once
#include "adsb/UserStructs/AdsbMsg.h"
#include "Planner/UserStructs/obstacle3D.h"

namespace Utils{

  void AdsbToObs(const UserStructs::AdsbMsg &adsb,
                 UserStructs::obstacle3D &obs3D);

}
