#include "DubinsLength2.h"
#include "OtherLibs/Dubins2D/dubins.h"
#include <cmath>
#include <iostream>

namespace Utils{

 double DubinsLength2(double x0,double y0,double z0,double yaw0,
                      double x1,double y1,double z1,double yaw1,
		      double r,double max_pitch)
 {
    DubinsPath path2D;

    double q0[]= { x0, y0, yaw0 };
    double q1[]= { x1, y1, yaw1 };
    dubins_init(q0, q1, r, &path2D);
    
    double length_2d= dubins_path_length(&path2D);
    return length_2d+ fabs(z1-z0)*tan(max_pitch/2);
 }

}
