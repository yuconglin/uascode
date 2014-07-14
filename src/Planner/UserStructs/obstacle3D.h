#pragma once
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <stdint.h>

namespace UserStructs
{//length unit: km
 //x: east, y: north
   struct obs3D {
            double x;
	    double y;
	    double z;
	    double r;
	    double hr;
	    obs3D( ):x(0),y(0),z(0),r(0),hr(0)
	    {
	    }
	    obs3D(double _x,double _y,double _z,double _r,double _hr):x(_x),y(_y),z(_z),r(_r),hr(_hr)
	    {
	    }
   };

   struct obstacle3D{
            uint32_t address;

            double x1,x2,x3;
	    double head_xy;//heading in xy convention
	    double speed;
	    double v_vert;
	    //double v1,v2,v3;
	    double t;
	    double r,dr;
	    double hr,dhr;
	    
	    //constructor
	    //default
	    obstacle3D():x1(0),x2(0),x3(0),head_xy(0),speed(0),v_vert(0),t(0),r(0),dr(0),hr(0),dhr(0){ };
	    
	    //short
	    //obstacle3D(double _x1, double _x2, double _head_xy, double _speed, double _x3, double _v_vert, double _t):
            //x1(_x1),x2(_x2),x3(_x3),head_xy(_head_xy),speed(_speed),v_vert(_v_vert),t(_t),r(0.6408),dr(0.1),hr(0.1524),dhr(0.025){ };
	    
	    //full
	    obstacle3D(double _x1, double _x2, double _head_xy, double _speed, double _x3, double _v_vert, double _t, double _r, double _dr,double _hr,double _dhr):
            x1(_x1),x2(_x2),x3(_x3),head_xy(_head_xy),speed(_speed),v_vert(_v_vert),t(_t),r(_r),dr(_dr),hr(_hr),dhr(_dhr){ };
            
	    obs3D Estimate(double t1) const
	    {
               if(t1<t)
	       {
		 std::cout<<"neg time "<<t1<<" "<< t<<std::endl;
		 try {
                   throw std::runtime_error ("negtive time");
                 }
                 catch (std::runtime_error &e){
                   std::cout << "Caught a runtime_error exception: "
                   << e.what () << '\n';
                 }

	       }
	       double x=x1+speed*cos(head_xy)*(t1-t);
               double y=x2+speed*sin(head_xy)*(t1-t);
	       double z=x3+v_vert*(t1-t);

	       return obs3D(x,y,z,r+dr,hr+dhr);
	    }

    };//struct ends

}//namespace ends
