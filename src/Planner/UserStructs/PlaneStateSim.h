#pragma once
//#include "common/UserStructs/constants.h"
#include "common/Utils/UTMtransform.h"
#include "math.h"
namespace UserStructs{

  struct PlaneStateSim{//NED , 
       double t;//sec
       double x;//m easting
       double y;//m northing
       double lat;
       double lon;
       double z;//m
       double speed;//m/s,ground speed
       double yaw;//rad,north zero
       double pitch;//rad
       double ax;//m/s^2
       double ay;//m/s^2
       double az;//m/s^2
       
       //constructor
       PlaneStateSim();
       PlaneStateSim(double _t,double _x,double _y,double _lat,double _lon,double _z,double _speed,double _yaw,double _pitch,double _ax,double _ay,double _az);

       PlaneStateSim SmallChange(double dt);

       void GetUTM();
       
       void GetGCS();
       
       void CheckConvert();

  };//struct ends

  struct StateNode{
    PlaneStateSim state;
    double length;

    StateNode():state(PlaneStateSim()),length(0)
    {
    }

    StateNode(PlaneStateSim _st,double _len):
      state(_st),length(_len){
    }

  };//struct StateNode ends

};
