#pragma once
#include "common/Utils/UTMtransform.h"
#include "Planner/Utils/PointInPoly.h"

#include "point2D.h"  
//std
#include <fstream>
#include <sstream>
#include <iomanip>
#include <iostream>

namespace UserStructs{
 
  struct SpaceLimit{
     //height upper limit
     double h_upper;
     double h_lower;
     //vertex for the geofencing polygon
     std::vector<point2D> vertex;
     //vertex for polygon
     //constructor
     SpaceLimit();

     ~SpaceLimit();
     
     SpaceLimit(double _h_upper,double _h_lower,const std::vector<point2D> _vertex);

    
     SpaceLimit(double _h_upper,double _h_lower);

     //to judge if a point is within the SpaceLimit
     bool TellIn(double x, double y,double z);
  
     //to load geofence from a file of (lat, lon)
     void LoadGeoFence(const char* filename);

  };//SpaceLimit ends

}//namespace ends
