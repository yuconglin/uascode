#include "PointInPoly.h"
#include "Planner/UserStructs/point2D.h"
#include <iostream>

using namespace UserStructs;

int main(int argc,char** argv)
{
  std::vector<point2D> vertex;
  vertex.push_back(point2D(0,0));
  vertex.push_back(point2D(5,0));
  vertex.push_back(point2D(5,5));
  vertex.push_back(point2D(0,5));

  std::cout<<Utils::PointInPoly(vertex,2,2)
          << std::endl;
  
}
