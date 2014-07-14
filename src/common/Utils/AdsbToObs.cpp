#include "AdsbToObs.h"
#include "common/Utils/UTMtransform.h"
#include "common/UserStructs/constants.h"
#include "common/Utils/GetTimeUTC.h"
#include "common/Utils/GeoUtils.h"
//std
#include <cmath>

namespace Utils{

void AdsbToObs(const UserStructs::AdsbMsg &adsb,
                 UserStructs::obstacle3D &obs)
{
  obs.address= adsb.address;
  Utils::ToUTM(adsb.longitude, adsb.latitude, obs.x1, obs.x2);
  obs.x3= adsb.altitude * UasCode::feet2meter; 
  obs.head_xy= Utils::_wrap_pi(M_PI/2- adsb.hd * UasCode::DEG2RAD);
  obs.speed= adsb.v * UasCode::kt2ms;
  obs.v_vert= adsb.vv * UasCode::feet2meter/60; 
  obs.t= Utils::GetTimeUTC();
  obs.r= 2100 * UasCode::feet2meter;
  obs.hr= 500 * UasCode::feet2meter;
  obs.dr= 0.;
  obs.dhr= 0.;
}//AdsbToObs ends

};
