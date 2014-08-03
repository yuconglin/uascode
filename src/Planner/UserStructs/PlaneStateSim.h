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
       PlaneStateSim():t(0),x(0),y(0),lat(0),lon(0),z(0),
                speed(0),yaw(0),pitch(0),ax(0),ay(0),az(0){ };

       PlaneStateSim(double _t,double _x,double _y,double _lat,double _lon,double _z,double _speed,double _yaw,double _pitch,double _ax,double _ay,double _az):
	 t(_t),x(_x),y(_y),lat(_lat),lon(_lon),z(_z),speed(_speed),
	 yaw(_yaw),pitch(_pitch),ax(_ax),ay(_ay),az(_az){ };

       PlaneStateSim SmallChange(double dt){
           PlaneStateSim st;
           st.t= t+dt;
           st.x= x+speed*cos(pitch)*sin(yaw)*dt;
           st.y= y+speed*cos(pitch)*cos(yaw)*dt;
           st.z= z+speed*sin(pitch)*dt;
           st.GetGCS();
           st.yaw= yaw;
           st.pitch= pitch;
           st.ax= ax;
           st.ay= ay;
           st.az= az;
           return st;
       }

       void GetUTM(){
         Utils::ToUTM(lon,lat,x,y);
       }
       
       void GetGCS(){
         Utils::FromUTM(x,y,lon,lat);
       }
       
       void CheckConvert(){
         if(lat==0 && lon==0){
	     GetGCS();
	     return;
	 }
	 if(x==0 && y==0){
	     GetUTM();
	     return;
	 }
       }

       	
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
