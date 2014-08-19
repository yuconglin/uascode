#include "CheckCollision.h"
#include "common/Utils/YcLogger.h"

#include "iostream"
#include <iomanip>

#include <cmath>

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.CheckCollision.YcLogger"));
}

namespace Utils{
 
int CheckCollision(const UserStructs::PlaneStateSim& plane, UserStructs::obstacle3D& obs)
{
  UserStructs::obs3D obs3d= obs.Estimate(plane.t);
  /*
  UASLOG(s_logger,LL_DEBUG,"obs3d: " << obs3d.x << " "
         << obs3d.y << " "<< obs3d.z);

  UASLOG(s_logger,LL_DEBUG,"plane: " << plane.x << " "
         << plane.y << " "<< plane.z);

  UASLOG(s_logger,LL_DEBUG,"check distance: "
         << sqrt(pow(obs3d.x-plane.x,2)+pow(obs3d.y-plane.y,2))<< " "
         << fabs(obs3d.z-plane.z) << " "
         << "plane time:" << std::setprecision(5) << std::fixed
         << plane.t
         ); */

  double uncertain_ratio = 1.2;

  if( fabs(obs3d.z-plane.z)> obs3d.hr ||
      sqrt(pow(obs3d.x-plane.x,2)+pow(obs3d.y-plane.y,2)) > uncertain_ratio*obs3d.r
    )
    return 0;
      
  return 1;

}//ends

int CheckCollision2(const UserStructs::PlaneStateSim &plane, UserStructs::obstacle3D &obs, double thres_ratio)
{
   UserStructs::obs3D obs3d= obs.Estimate(plane.t);

   UASLOG(s_logger,LL_DEBUG,"check distance: "
          << sqrt(pow(obs3d.x-plane.x,2)+pow(obs3d.y-plane.y,2))<< " "
          << fabs(obs3d.z-plane.z) << " "
          << "plane time:" << std::setprecision(5) << std::fixed
          << plane.t
          << "obs r:" << obs3d.r << " "
          << "obs hr:"<< obs3d.hr
          );

   int result;
   if( fabs(obs3d.z-plane.z)> obs3d.hr ||
       sqrt(pow(obs3d.x-plane.x,2)+pow(obs3d.y-plane.y,2)) > thres_ratio*obs3d.r
     )
   {
     result=0;
     UASLOG(s_logger,LL_DEBUG,"beyond");
   }
   else
   {
     result=1;
     UASLOG(s_logger,LL_DEBUG,"inside");
   /*
   UASLOG(s_logger,LL_DEBUG,"colli check distance: "
          << sqrt(pow(obs3d.x-plane.x,2)+pow(obs3d.y-plane.y,2))<< " "
          << fabs(obs3d.z-plane.z) << " "
          << "plane time:" << std::setprecision(5) << std::fixed
          << plane.t
          );*/
   }
   return result;
}

}
