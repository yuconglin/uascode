#pragma once
#include "Planner/UserStructs/obstacle3D.h"
#include "Planner/UserStructs/PlaneStateSim.h"

namespace Utils{
  //in planning, check the collision between a PlaneStateSim and a adsb obstacle
  //0: no collision, 1: collision
  int CheckCollision(const UserStructs::PlaneStateSim& plane, UserStructs::obstacle3D& obs);

  int CheckCollision2(const UserStructs::PlaneStateSim &plane, UserStructs::obstacle3D &obs, double thres_ratio);

  int CheckCollisionSet(const UserStructs::PlaneStateSim &plane,
                        UserStructs::obstacle3D &obs);
}
