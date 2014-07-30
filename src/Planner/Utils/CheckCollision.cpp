#include "CheckCollision.h"
#include "common/Utils/YcLogger.h"
#include "iostream"
#include <cmath>

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.CheckCollision.YcLogger"));
}

namespace Utils{
 
int CheckCollision(const UserStructs::PlaneStateSim& plane, UserStructs::obstacle3D& obs)
{
  
  UserStructs::obs3D obs3d= obs.Estimate(plane.t);
  /*
  UASLOG(s_logger,LL_DEBUG,"distance: "
         << sqrt(pow(obs3d.x-plane.x,2)+pow(obs3d.y-plane.y,2))<< " "
         << fabs(obs3d.z-plane.z)
         );*/

  if( fabs(obs3d.z-plane.z)> obs3d.hr ||
      sqrt(pow(obs3d.x-plane.x,2)+pow(obs3d.y-plane.y,2)) > obs3d.r
    )
    return 0;
      
  return 1;

}//ends

};
