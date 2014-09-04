#pragma once
#include "armadillo"

namespace UserStructs{
  struct PlaneStateSim;
  struct MissionSimPt;
}

namespace Utils{

float get_bearing_to_next_waypoint(double lat_now, double lon_now, double lat_next, double lon_next);

float _wrap_pi(float bearing);

float get_vec_cross(const arma::vec::fixed<2> &vec1,const arma::vec::fixed<2> &vec2);

double get_angle(const arma::vec::fixed<2> &loc, const arma::vec::fixed<2> &pt1, const arma::vec::fixed<2> &pt2);

double get_distance(const arma::vec::fixed<2>& pt1, const arma::vec::fixed<2>& pt2);

bool location_passed_pt(const arma::vec::fixed<2> &loc, const arma::vec::fixed<2> &pt1, const arma::vec::fixed<2> &pt2);

bool location_passed_point(const UserStructs::PlaneStateSim& st_now,const arma::vec::fixed<2>& pt_A,const UserStructs::MissionSimPt& pt_target);
}
