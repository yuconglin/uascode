#pragma once

#include "Planner/UserStructs/point2D.h"
//#include "Planner/UserStructs/obstacle3D.h"

namespace Utils{
  void CreateSetPoints( UserStructs::obstacle3D &obs, double t,
                        std::vector<UserStructs::point2D>& set_points);
}
