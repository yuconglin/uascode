#pragma once
#include "armadillo"

namespace Utils{

float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

float _wrap_pi(float bearing);

float get_vec_cross(const arma::vec::fixed<2> &vec1,const arma::vec::fixed<2> &vec2);
};
