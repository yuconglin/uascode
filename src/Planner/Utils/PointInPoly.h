#pragma once
#include "Planner/UserStructs/point2D.h"
#include <vector>
namespace Utils{

  bool PointInPoly(std::vector<UserStructs::point2D> vertex,double x,double y);

};//namespaces ends
