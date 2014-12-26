#include "Planner/UserTypes/ReachabilitySet.hpp"
#include "common/UserStructs/constants.h"
#include <cmath>
#include <cassert>
#include <vector>

namespace UasCode{
  
  ReachabilitySet::ReachabilitySet(const UserStructs::obstacle3D& _obs) {
    spd = _obs.speed + 10;
    omiga = ( UasCode::CONSTANT_G * tan(25/180*M_PI) / spd ) < 3*M_PI/180. ? UasCode::CONSTANT_G * tan(25/180*M_PI) / spd : 3*M_PI/180. ;
    rho = spd / omiga;
    hd = _obs.head_xy;
    x0 = _obs.x1;
    y0 = _obs.x2;
    t0 = _obs.t;
  }

  ReachabilitySet::GetSet(int num, double t)
  {
     //first get the points
     set_points.clear();
     double d_the = 2*M_PI/num;
     for(int i=0;i!= num+1;++i){
         double the= -M_PI + d_the * i;
         set_points.push_back( UserStructs::point2D(x(the,t),y(the,t)) );
     }

     //and then converts to obstacle's reference frame
     for(int i=0;i!= set_points.size();++i){
        double x1 = set_points[i].x * cos(hd-M_PI/2) + set_points[i].y * sin(hd-M_PI/2) + x0;
        double y1 = -set_points[i].x * sin(hd-M_PI/2) + set_points[i].y * cos(hd-M_PI/2) + y0;
        set_points[i].x = x1;
        set_points[i].y = y1;
     }

  }

  ReachabilitySet::x1(double the, double t){
    return -rho*(1-cos(the) )+(spd*t+rho*the+r)*sin(the);
  }

  ReachabilitySet::y1(double the, double t){
    return -rho*sin(the)+(spd*t+rho*the+r)*cos(the);
  }

  ReachabilitySet::x2(double the, double t){
    return rho*(1-cos(the))+(spd*t-rho*the+r)*sin(the);
  }

  ReachabilitySet::y2(double the, double t){
    return rho*sin(the)+(spd*t-rho*the+r)*cos(the);
  }

  ReachabilitySet::x3(double the, double t){
    return -rho*(1-cos(omiga*t))+r*sin(the);
  }

  ReachabilitySet::y3(double the, double t){
    return rho*sin(omiga*t)+r*cos(the);
  }

  ReachabilitySet::x4(double the, double t){
    return rho*(1-cos(omiga*t))+r*sin(the);
  }

  ReachabilitySet::y4(double the, double t){
    return rho*sin(omiga*t)+r*cos(the);
  }

  ReachabilitySet::x(double the, double t){
    //y-axis or north is zero degree direction
    assert( -M_PI <= the && the <= M_PI);
    double xs;
    if(omiga*t <= M_PI){
	    if(-omiga*t <= the && the <=0){
	       xs = x1(the,t);
	    }
	    else if (0<= the && the <= omiga*t){
	       xs = x2(the,t);
	    }
	    else if (M_PI <= the && the <= -omiga*t){
	       xs = x3(the,t);
	    }
	    else{
	       xs = x4(the,t);
	    }
    }
    else{
            if( M_PI <= the && the <= 0){
	       xs = x1(the,t);
	    }
	    else{
	       xs = x2(the,t);
	    }
    }
    return xs;
  }

  ReachabilitySet::y(double the, double t){
    //y-axis or north is zero degree direction
    assert( -M_PI <= the && the <= M_PI);
    double ys;
    if(omiga*t <= M_PI){
         if(-omiga*t <= the && the <=0){
	    ys = y1(the,t);
	 }
	 else if (0<= the && the <= omiga*t){
	    ys = y2(the,t);
	 }
	 else if (M_PI <= the && the <= -omiga*t){
	    ys = y3(the,t);
	 }
	 else{
	    ys = y4(the,t);
	 }
    }
    else{
         if( M_PI <= the && the <= 0){
	    ys = y1(the,t);
         }
         else{
	    ys = y2(the,t);
         }
    }
    return ys;
  }

}
