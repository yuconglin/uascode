/********
//struct mission_item_s {
	bool altitude_is_relative;	/**< true if altitude is relative from start point	*/
//	double lat;			/**< latitude in degrees				*/
//	double lon;			/**< longitude in degrees				*/
//	float altitude;			/**< altitude in meters					*/
//	float yaw;			/**< in radians NED -PI..+PI, NAN means don't change yaw		*/
//	float loiter_radius;		/**< loiter radius in meters, 0 for a VTOL to hover     */
//	int8_t loiter_direction;	/**< 1: positive / clockwise, -1, negative.		*/
//	enum NAV_CMD nav_cmd;		/**< navigation command					*/
//	float acceptance_radius;	/**< default radius in which the mission is accepted as reached in meters */
//	float time_inside;		/**< time that the MAV should stay inside the radius before advancing in seconds */
//	float pitch_min;		/**< minimal pitch angle for fixed wing takeoff waypoints */
//	bool autocontinue;		/**< true if next waypoint should follow after this one */
//	enum ORIGIN origin;		/**< where the waypoint has been generated		*/
//};
#pragma once
#include "common/Utils/UTMtransform.h" 
#include <cmath>
namespace UserStructs{

struct MissionSimPt{
  double lat;
  double lon;
  float alt;
  float yaw;//yaw in traditional x-y convention
  float r;//meter
  //other
  double x;//UTM in meters
  double y;//x easting, y northing
  double h_rec;//horizontal length of the rectangle, in meter
  double v_rec;//vertical length of the associated rectangle
  double alt_rec;//alttitude. acutally, when arrived, the uav should be 0.1h_rec,0.1v_rec,0.5alt 

  //constructor
  MissionSimPt():lat(0.),lon(0.),alt(0.),yaw(0.),r(0.),x(0.),y(0.),h_rec(0.),v_rec(0.),alt_rec(0.){};
  
  MissionSimPt(double _lat,double _lon,float _alt,float _yaw,float _r,double _x,double _y,double _h,double _v,double _alt_rec):lat(_lat),lon(_lon),alt(_alt),yaw(_yaw),r(_r),x(_x),y(_y),h_rec(_h),v_rec(_v),alt_rec(_alt_rec){ };

  void GetUTM(){
   Utils::ToUTM(lon,lat,x,y);
 }
 
 void GetGCS(){
   Utils::FromUTM(x,y,lon,lat);
 }

 void CheckConvert(){
   if(x==0&&y==0) {
     GetUTM();
     return;
   }
   if(lat==0&&lon==0) {
     GetGCS();
     return;
   }

 }

 bool SeeArrive(double x1, double y1, double z1){
   if(fabs(x1-x)<= 0.1*h_rec && 
      fabs(y1-y)<= 0.1*v_rec && 
      fabs(z1-alt)<= 0.5*alt_rec
     )
     return true;
   return false;
 }

 bool SeeArriveSphere(double x1,double y1,double z1){
   double dis= sqrt( pow(x1-x,2)
                   + pow(y1-y,2)
		   + pow(z1-alt,2)
       );
   if(dis< r) return true;
   return false;
 }

};
};
