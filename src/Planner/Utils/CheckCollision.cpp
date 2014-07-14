#include "CheckCollision.h"
#include <cmath>

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

};
