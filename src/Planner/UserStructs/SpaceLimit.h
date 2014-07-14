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
     SpaceLimit():h_upper(0.),h_lower(0.){};

     ~SpaceLimit(){};
     
     SpaceLimit(double _h_upper,double _h_lower,const std::vector<point2D> _vertex)
     :h_upper(_h_upper),h_lower(_h_lower),vertex(_vertex){};//SpaceLimit ends
    
     SpaceLimit(double _h_upper,double _h_lower):
       h_upper(_h_upper),h_lower(_h_lower){}

     //to judge if a point is within the SpaceLimit
     bool TellIn(double x, double y,double z)
     { //in return true;
       if( z<= h_lower ||z>= h_upper){ 
	 std::cout<<"not in height"<< std::endl;
	 return false;
       }
       if( !Utils::PointInPoly(vertex,x,y) ){
	 std::cout<<"not in poly" << std::endl;
	 return false;
       }
       return true;
     }//TellIn ends
  
     //to load geofence from a file of (lat, lon)
     void LoadGeoFence(const char* filename)
     {
       std::cout<<"load geofence"<< std::endl;
       std::fstream myfile(filename);
       if(myfile.is_open() ){
         std::string line;
	 while(std::getline(myfile,line) )
	 {
           std::istringstream iss(line);
	   double lat,lon,x,y;
	   iss >> lat >> lon;
           Utils::ToUTM(lon,lat,x,y);
	   vertex.push_back(UserStructs::point2D(x,y) );
	   std::cout<<x<<" "<<y<< std::endl;
	 } //while ends
       }//open ends
       else
	std::cerr << "error:geofence file not found\n";
     }//LoadGeoFence ends

  };//SpaceLimit ends

}//namespace ends
