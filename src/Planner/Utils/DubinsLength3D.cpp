#include "DubinsLength3D.h"
#include "OtherLibs/Dubins2D/dubins.h"
#include <cmath>
#include <iostream>

namespace Utils{

 double DubinsLength3D(double x0,double y0,double z0,double yaw0,
                       double x1,double y1,double z1,double yaw1,
		       double r)
 {
    std::cout<< "DubinsLength3D"<< std::endl;
    std::cout<< x0<<" "<<y0<<" "<<z0<<" "<<yaw0/M_PI*180.<< std::endl;
    std::cout<< x1<<" "<<y1<<" "<<z1<<" "<<yaw1/M_PI*180.<< std::endl;
    std::cout<<"rho: "<< r<< std::endl;
    DubinsPath path2D;

    double q0[]= { x0, y0, yaw0 };
    double q1[]= { x1, y1, yaw1 };
    dubins_init(q0, q1, r, &path2D);
    
    double length_2d= dubins_path_length(&path2D);
 
    return sqrt(length_2d*length_2d+(z0-z1)*(z0-z1) );
 }

}
