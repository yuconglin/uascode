#include "Planner/UserTypes/ReachabilitySet.hpp"
#include "common/UserStructs/constants.h"
#include "Planner/Utils/PointInPoly.h"
#include "common/Utils/YcLogger.h"
#include "Planner/UserStructs/SetPointsVh.h"

#include <cmath>
#include <cassert>
#include <vector>
#include <fstream>
#include <iomanip>

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.ReachabilitySets.YcLogger"));
}

namespace UasCode{
  
  ReachabilitySet::ReachabilitySet(const UserStructs::obstacle3D& _obs) {
    spd = _obs.speed;
    omiga = ( UasCode::CONSTANT_G * tan(25./180*M_PI) / spd ) < 3*M_PI/180. ? UasCode::CONSTANT_G * tan(25./180*M_PI) / spd : 3*M_PI/180. ;
    rho = spd / omiga;
    hd = _obs.head_xy;
    x0 = _obs.x1;
    y0 = _obs.x2;
    t0 = _obs.t;
    z0 = _obs.x3;
    vert = _obs.v_vert;
    r = _obs.r;
    hr = _obs.hr;
    this->up_the = M_PI / 16.;
  }

  void ReachabilitySet::GetSet(int num, double t)
  {
     assert(t>=t0);
     //first get the points
     set_points.clear();
     double d_the = 2*M_PI/num;
     for(int i= 0;i!= num+1;++i){
         double the = -M_PI + d_the * i;

         if( i == 0){
             the = -M_PI;
         }

         if ( i == num ){
             the = M_PI;
         }

         set_points.push_back( UserStructs::point2D(x(the,t-t0),y(the,t-t0)) );
     }

     //and then converts to obstacle's reference frame
     for(int i=0;i!= set_points.size();++i){
        double x1 = set_points[i].x * cos(hd-M_PI/2) - set_points[i].y * sin(hd-M_PI/2) + x0;
        double y1 = set_points[i].x * sin(hd-M_PI/2) + set_points[i].y * cos(hd-M_PI/2) + y0;
        set_points[i].x = x1;
        set_points[i].y = y1;
     }

     //get vertical constraints
     h_low = z0 + (vert-10) * (t-t0) - hr;
     h_high = z0 + (vert+10) * (t-t0) + hr;
  }

  UserStructs::SetPointsVh ReachabilitySet::AccessSet(){
     return UserStructs::SetPointsVh(set_points, h_low, h_high);
  }

  bool ReachabilitySet::InSet(double x, double y)
  {
     return Utils::PointInPoly(this->set_points,x,y);
  }

  bool ReachabilitySet::InSet3(double x, double y, double z)
  {
      //UASLOG(s_logger,LL_DEBUG,"in 2d ?"
      //       << InSet(x,y) );

      return InSet(x,y) && z < h_high && z > h_low;
  }

  void ReachabilitySet::OutputSet(const char *filename)
  {
     std::ofstream file( filename );
     for(int i=0; i!= set_points.size(); ++i){
        file << set_points[i].x << " " << set_points[i].y << '\n';
     }//
  }

  //private functions
  double ReachabilitySet::x1(double the, double t){
    return -rho*(1-cos(the) )+(spd*t+rho*the+r)*sin(the);
  }

  double ReachabilitySet::y1(double the, double t){
    return -rho*sin(the)+(spd*t+rho*the+r)*cos(the);
  }

  double ReachabilitySet::x2(double the, double t){
    return rho*(1-cos(the))+(spd*t-rho*the+r)*sin(the);
  }

  double ReachabilitySet::y2(double the, double t){
    return rho*sin(the)+(spd*t-rho*the+r)*cos(the);
  }

  double ReachabilitySet::x3(double the, double t){
    //return -rho*(1-cos(omiga*t))+r*sin(the);
    return -rho*(1-cos(up_the)) - (spd*t-rho*up_the)*sin(up_the) + r*sin(the);
  }

  double ReachabilitySet::y3(double the, double t){
    //return rho*sin(omiga*t)+r*cos(the);
    return rho*sin(up_the) + (spd*t-rho*up_the)*cos(up_the) + r*cos(the);
  }

  double ReachabilitySet::x4(double the, double t){
    //return rho*(1-cos(omiga*t))+r*sin(the);
    return rho*(1-cos(up_the)) + (spd*t-rho*up_the)*sin(up_the) + r*sin(the);
  }

  double ReachabilitySet::y4(double the, double t){
    //return rho*sin(omiga*t)+r*cos(the);
    return rho*sin(up_the) + (spd*t-rho*up_the)*cos(up_the) + r*cos(the);
  }

  double ReachabilitySet::x(double the, double t){
    //y-axis or north is zero degree direction
    assert( -M_PI <= the && the <= M_PI);
    double xs;
    if(up_the <= M_PI){
	    if(-up_the <= the && the <=0){
	       xs = x1(the,t);
        }
	    else if (0<= the && the <= up_the){
	       xs = x2(the,t);
	    }
	    else if (-M_PI <= the && the <= -up_the){
	       xs = x3(the,t);
	    }
	    else{
	       xs = x4(the,t);
	    }
    }
    else{
            if( -M_PI <= the && the <= 0){
	       xs = x1(the,t);
	    }
	    else{
	       xs = x2(the,t);
	    }
    }
    return xs;
  }

  double ReachabilitySet::y(double the, double t){
    //y-axis or north is zero degree direction
    assert( -M_PI <= the && the <= M_PI);
    double ys;
    if(up_the <= M_PI){
         if(-up_the <= the && the <=0){
	    ys = y1(the,t);
	 }
	 else if (0<= the && the <= up_the){
	    ys = y2(the,t);
	 }
	 else if (-M_PI <= the && the <= -up_the){
	    ys = y3(the,t);
	 }
	 else{
	    ys = y4(the,t);
	 }
    }
    else{
         if( -M_PI <= the && the <= 0){
	    ys = y1(the,t);
         }
         else{
	    ys = y2(the,t);
         }
    }
    return ys;
  }

}
