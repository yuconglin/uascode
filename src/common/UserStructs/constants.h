#pragma once
#include <string>
namespace UasCode{
 const double DEG2RAD= 0.0174532925;
 const double R_EARTH=6371000;//meters	
 const double CONSTANT_G=9.80665;
 const std::string ZONE= "12N"; //12N for AZ, 16N for IN 
 const double feet2meter= 0.3048;
 const double kt2ms= 0.514444444;
 const int MAVLINK_IN_PORT = 14550;
 const std::string MAVLINK_OUT_PORT = "19550";
 const int ADSB_IN_PORT = 4000;
}
