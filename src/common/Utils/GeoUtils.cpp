#include "GeoUtils.h"
#include "common/UserStructs/constants.h"
//standard
#include <cmath>
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

}
