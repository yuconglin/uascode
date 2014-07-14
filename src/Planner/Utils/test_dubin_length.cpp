#include "DubinsLength3D.h"
#include <cmath>
#include <iostream>

int main(int argc,char** argv)
{
  double x0= 100, y0=100, z0= 20, yaw0= M_PI/4;
  double x1= 200, y1=100, z1= 30, yaw1= M_PI/2;
  std::cout<<"dubin length: "<< Utils::DubinsLength3D(x0,y0,z0,yaw0,
                                          x1,y1,z1,yaw1,50)
                             << std::endl;
  return 0;
}
