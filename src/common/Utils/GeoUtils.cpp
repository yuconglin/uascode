#include "GeoUtils.h"
#include "common/UserStructs/constants.h"
#include "UserStructs/MissionSimPt.h"
#include "UserStructs/PlaneStateSim.h"
#include "common/Utils/UTMtransform.h"
//standard
#include "armadillo"
#include <cmath>
#include <cfloat>

namespace Utils{
  
float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next)
{
	double lat_now_rad = lat_now * UasCode::DEG2RAD;
	double lon_now_rad = lon_now * UasCode::DEG2RAD;
	double lat_next_rad = lat_next * UasCode::DEG2RAD;
	double lon_next_rad = lon_next * UasCode::DEG2RAD;

	double d_lat = lat_next_rad - lat_now_rad;
	double d_lon = lon_next_rad - lon_now_rad;

	/* conscious mix of double and float trig function to maximize speed and efficiency */
	float theta = atan2f(sin(d_lon) * cos(lat_next_rad) , cos(lat_now_rad) * sin(lat_next_rad) - sin(lat_now_rad) * cos(lat_next_rad) * cos(d_lon));

	theta = _wrap_pi(theta);

	return theta;
}

float _wrap_pi(float bearing)
{
	int c = 0;

	while (bearing > M_PI && c < 30) {
		bearing -= 2*M_PI;
		c++;
	}

	c = 0;

	while (bearing <=  -M_PI && c < 30) {
		bearing += 2*M_PI;
		c++;
	}

	return bearing;
}

float get_vec_cross(const arma::vec::fixed<2> &vec1,const arma::vec::fixed<2> &vec2)
{
   return vec1(0)*vec2(1)-vec2(0)*vec1(1);
}

double get_angle(const arma::vec::fixed<2> &loc, const arma::vec::fixed<2> &pt1, const arma::vec::fixed<2> &pt2)
{
    double mod_vec1= std::sqrt(pow(pt2(0)-pt1(0),2)+pow(pt2(1)-pt1(1),2));
    double mod_vec2= std::sqrt(pow(pt2(0)-loc(0),2)+pow(pt2(1)-loc(1),2));
    double dot = (pt2(0)-pt1(0))*(pt2(0)-loc(0)) + (pt2(1)-pt1(1))*(pt2(1)-loc(1));

    if (mod_vec1 == 0 || mod_vec2 == 0) return DBL_MAX;

    return acos(dot/(mod_vec1*mod_vec2));
}

double get_distance(const arma::vec::fixed<2>& pt1, const arma::vec::fixed<2>& pt2)
{
    return std::sqrt(pow(pt2(0)-pt1(0),2)+pow(pt2(1)-pt1(1),2));
}

bool location_passed_pt(const arma::vec::fixed<2> &loc, const arma::vec::fixed<2> &pt1, const arma::vec::fixed<2> &pt2)
{
    double angle = get_angle(loc,pt1,pt2);

    if(angle == DBL_MAX){
        if( get_distance(loc,pt2)==0 )
            return true;
        return false;
    }else if (angle == 0){
        return get_distance(loc,pt1) > get_distance(pt2,pt1);
    }

    if(angle > M_PI/2) return true;
    return false;
}

bool location_passed_point(const UserStructs::PlaneStateSim& st_now,const arma::vec::fixed<2>& pt_A,const UserStructs::MissionSimPt& pt_target)
{
    arma::vec::fixed<2> loc;
    arma::vec::fixed<2> pt1;
    arma::vec::fixed<2> pt2;

    loc << st_now.x << st_now.y;
    double x1,y1;
    Utils::ToUTM(pt_A(1),pt_A(0),x1,y1);
    pt1 << x1 << y1;
    pt2 << pt_target.x << pt_target.y;

    double angle = get_angle(loc,pt1,pt2);

    if(angle == DBL_MAX){
        if( get_distance(loc,pt2)==0 )
            return true;
        return false;
    }else if (angle == 0){
        return get_distance(loc,pt1) > get_distance(pt2,pt1);
    }

    if(angle > M_PI/2) return true;
    return false;
    return Utils::location_passed_pt(loc,pt1,pt2);
}

}
