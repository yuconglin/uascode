#include "DubinsLength2.h"
#include "OtherLibs/Dubins2D/dubins.h"
#include <cmath>
#include <iostream>

namespace Utils{

 double DubinsLength2(double x0,double y0,double z0,double yaw0,
                      double x1,double y1,double z1,double yaw1,
		      double r,double max_pitch)
 {
    std::cout<<"print DubinsLength2"<< std::endl;
    DubinsPath path2D;
    
    std::cout<<"x0: "<<x0<<" y0: "<< y0<<" z0: "<< z0<<" hd0: "<< yaw0/M_PI*180. << std::endl;
    std::cout<<"x1: "<<x1<<" y1: "<< y1<<" z1: "<< z1<<" hd1: "<< yaw1/M_PI*180. << std::endl;
    std::cout<<"rho: "<< r<< std::endl;

    double q0[]= { x0, y0, yaw0 };
    double q1[]= { x1, y1, yaw1 };
    dubins_init(q0, q1, r, &path2D);
    
    double length_2d= dubins_path_length(&path2D);
    std::cout<<"dubin length: "<< length_2d << std::endl;
    std::cout<<"other part: "<< fabs(z1-z0)*tan(max_pitch/2) << std::endl;
    std::cout<<"print DubinsLength2 ends"<< std::endl;

    return length_2d+ fabs(z1-z0)*tan(max_pitch/2);
 }

};
