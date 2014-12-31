#include "CheckCollision.h"
#include "common/Utils/YcLogger.h"
#include "Planner/UserStructs/point2D.h"
#include "Planner/UserTypes/ReachabilitySet.hpp"

#include "iostream"
#include <iomanip>
#include <vector>
#include <cmath>

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.CheckCollision.YcLogger"));
}

namespace Utils{
 
int CheckCollision(const UserStructs::PlaneStateSim& plane, UserStructs::obstacle3D& obs)
{
  UserStructs::obs3D obs3d= obs.Estimate(plane.t);

  if( fabs(obs3d.z-plane.z)> obs3d.hr ||
      sqrt(pow(obs3d.x-plane.x,2)+pow(obs3d.y-plane.y,2)) > obs3d.r
    )
    return 0;
      
  return 1;

}//ends

int CheckCollision2(const UserStructs::PlaneStateSim &plane, UserStructs::obstacle3D &obs, double thres_ratio)
{
   int result;
   /*
   obs.r *= thres_ratio;
   UasCode::ReachabilitySet set(obs);
   set.GetSet(20,plane.t);
   result = set.InSet3(plane.x,plane.y,plane.z) ? 1 : 0;
   */

   UserStructs::obs3D obs3d= obs.Estimate(plane.t);
   if( fabs(obs3d.z-plane.z)> obs3d.hr ||
       sqrt(pow(obs3d.x-plane.x,2)+pow(obs3d.y-plane.y,2)) > thres_ratio*obs3d.r
     )
   {
     result=0;
   }
   else
   {
     result=1;
   }
   return result;
}

int CheckCollisionSet(const UserStructs::PlaneStateSim &plane, UserStructs::obstacle3D &obs)
{
   UasCode::ReachabilitySet set(obs);
   set.GetSet(20,plane.t);

   int result = set.InSet3(plane.x,plane.y,plane.z) ? 1 : 0;

   if (result == 1){
       //std::cout << "t_diff:" << plane.t-obs.t << '\n';
       set.OutputSet("real_set.txt");
   }

   return result;
}

}
