#include "L1ControlSim.hpp"
#include "common/UserStructs/constants.h"
#include "common/Utils/GeoUtils.h"
#include "common/Utils/MathUtils.h"
//standard
#include <cmath>
#include "armadillo"
#include <algorithm>
#include <iostream>

namespace UasCode{
  
  L1ControlSim::L1ControlSim()
    :_L1_period(25),_L1_damping(0.75),_roll_lim_rad(30./180*M_PI)
  {
    /* calculate the ratio introduced in [2] */
    _L1_ratio = 1.0f / M_PI * _L1_damping * _L1_period;
    /* calculate normalized frequency for heading tracking */
    _heading_omega = sqrtf(2.0f) * M_PI / _L1_period;
   /* calculate the L1 gain (following [2]) */
    _K_L1 = 4.0f * _L1_damping * _L1_damping;

  }

  void L1ControlSim::set_l1_period(float period){
    _L1_period = period;
    /* calculate the ratio introduced in [2] */
    _L1_ratio = 1.0f / M_PI * _L1_damping * _L1_period;
    /* calculate normalized frequency for heading tracking */
    _heading_omega = sqrtf(2.0f) * M_PI / _L1_period;
  }

  void L1ControlSim::set_l1_damping(float damping) {
   _L1_damping = damping;
   /* calculate the ratio introduced in [2] */
   _L1_ratio = 1.0f / M_PI * _L1_damping * _L1_period;
   /* calculate the L1 gain (following [2]) */
   _K_L1 = 4.0f * _L1_damping * _L1_damping;
  }

  void L1ControlSim::set_l1_roll_limit(float roll_lim_rad) { 
   _roll_lim_rad = roll_lim_rad;
  }

  float L1ControlSim::nav_roll()
  {
	float ret = atanf(_lateral_accel * 1.0f / CONSTANT_G);
	ret = Utils::math::constrain(ret, -M_PI/2, M_PI/2);
	return ret;
  }

  float L1ControlSim::nav_bearing()
  {
     return Utils::_wrap_pi(_nav_bearing);
  }

  arma::vec::fixed<2> L1ControlSim::get_local_planar_vector(const arma::vec::fixed<2> &origin, const arma::vec::fixed<2> &target) const
  {
  /* this is an approximation for small angles, proposed by [2] */
  //math::Vector<2> out(math::radians((target(0) - origin(0))), math::radians((target(1) - origin(1))*cosf(math::radians(origin(0)))));
  //return out * static_cast<float>(CONSTANTS_RADIUS_OF_EARTH);
  arma::vec::fixed<2> out;
  out<<(target(0)-origin(0))*DEG2RAD<<(target(1)-origin(1))*DEG2RAD;
  return out*static_cast<float>(R_EARTH);
 }

 void L1ControlSim::navigate_waypoints(const arma::vec::fixed<2> &pt_A,const arma::vec::fixed<2> &pt_B,const arma::vec::fixed<2> &curr_position,const arma::vec::fixed<2> &gnd_speed)
{
   
   float eta;
   float xtrack_vel;
   float ltrack_vel;
   
   //get direction between the last (visited) and next waypoint
   _target_bearing= Utils::get_bearing_to_next_waypoint(curr_position(0),curr_position(1),pt_B(0),pt_B(1) );
   /* enforce a minimum ground speed of 0.1 m/s to avoid singularities */
  float ground_speed = std::max(arma::norm(gnd_speed,2), 0.1);

  /* calculate the L1 length required for the desired period */
  _L1_distance = _L1_ratio * ground_speed;

  /* calculate vector from A to B */
  arma::vec::fixed<2> vector_AB = get_local_planar_vector(pt_A, pt_B);
/*
   * check if waypoints are on top of each other. If yes,
   * skip A and directly continue to B
   */
  if (arma::norm(vector_AB,2) < 1.0e-6f) {
	  vector_AB = get_local_planar_vector(curr_position, pt_B);
  }

  vector_AB= arma::normalise(vector_AB);
/* calculate the vector from waypoint A to the aircraft */
  arma::vec::fixed<2> vector_A_to_airplane = get_local_planar_vector(pt_A,curr_position);

  /* calculate crosstrack error (output only) */
  _crosstrack_error = Utils::get_vec_cross(vector_AB,vector_A_to_airplane);
  /*
   * If the current position is in a +-135 degree angle behind waypoint A
   * and further away from A than the L1 distance, then A becomes the L1 point.
   * If the aircraft is already between A and B normal L1 logic is applied.
   */
  float distance_A_to_airplane = arma::norm(vector_A_to_airplane,2);
  float alongTrackDist = arma::dot(vector_A_to_airplane,vector_AB);

  /* estimate airplane position WRT to B */
  arma::vec::fixed<2> vector_B_to_P_unit = arma::normalise(get_local_planar_vector(pt_B, curr_position));
  
  /* calculate angle of airplane position vector relative to line) */

  // XXX this could probably also be based solely on the dot product
  float AB_to_BP_bearing = atan2f( Utils::get_vec_cross(vector_B_to_P_unit,vector_AB), arma::dot(vector_B_to_P_unit,vector_AB) );
  
  /* extension from [2], fly directly to A */
  if (distance_A_to_airplane > _L1_distance && alongTrackDist / std::max(distance_A_to_airplane , 1.0f) < -0.7071f) {

	/* calculate eta to fly to waypoint A */

	/* unit vector from waypoint A to current position */
	arma::vec::fixed<2> vector_A_to_airplane_unit = arma::normalise(vector_A_to_airplane);
	/* velocity across / orthogonal to line */
	xtrack_vel = Utils::get_vec_cross(gnd_speed,-1*vector_A_to_airplane_unit);
	/* velocity along line */
	ltrack_vel = arma::dot(gnd_speed,-1*vector_A_to_airplane_unit);
	eta = atan2f(xtrack_vel, ltrack_vel);
	/* bearing from current position to L1 point */
	_nav_bearing = atan2f(-1*vector_A_to_airplane_unit(1) , -1*vector_A_to_airplane_unit(0));

  /*
   * If the AB vector and the vector from B to airplane point in the same
   * direction, we have missed the waypoint. At +- 90 degrees we are just passing it.
   */
  } else if (fabsf(AB_to_BP_bearing) < DEG2RAD*100.0f) {
	  /*
	   * Extension, fly back to waypoint.
	   * 
	   * This corner case is possible if the system was following
	   * the AB line from waypoint A to waypoint B, then is
	   * switched to manual mode (or otherwise misses the waypoint)
	   * and behind the waypoint continues to follow the AB line.
	   */
	  /* calculate eta to fly to waypoint B */
	  
	  /* velocity across / orthogonal to line */
	  xtrack_vel = Utils::get_vec_cross(gnd_speed,-1*vector_B_to_P_unit);
	  /* velocity along line */
	  ltrack_vel = arma::dot(gnd_speed,-1*vector_B_to_P_unit);
	  eta = atan2f(xtrack_vel, ltrack_vel);
	  /* bearing from current position to L1 point */
	  _nav_bearing = atan2f(-vector_B_to_P_unit(1) , -vector_B_to_P_unit(0) );

  } else {
	  /* calculate eta to fly along the line between A and B */

	  /* velocity across / orthogonal to line */
	  xtrack_vel = Utils::get_vec_cross(gnd_speed,vector_AB);
	  /* velocity along line */
	  ltrack_vel = arma::dot(gnd_speed, vector_AB);
	  /* calculate eta2 (angle of velocity vector relative to line) */
	  float eta2 = atan2f(xtrack_vel, ltrack_vel);
	  /* calculate eta1 (angle to L1 point) */
	  float xtrackErr = Utils::get_vec_cross(vector_A_to_airplane,vector_AB);
	  float sine_eta1 = xtrackErr / std::max(_L1_distance , 0.1f);
	  /* limit output to 45 degrees */
	  sine_eta1 = Utils::math::constrain(sine_eta1, -0.7071f, 0.7071f); //sin(pi/4) = 0.7071
	  float eta1 = asinf(sine_eta1);
	  eta = eta1 + eta2;
	  /* bearing from current position to L1 point */
	  _nav_bearing = atan2f(vector_AB(1), vector_AB(0)) + eta1;

  }

  /* limit angle to +-90 degrees */
  eta = Utils::math::constrain(eta, -M_PI/ 2.0f, M_PI / 2.0f);
  _lateral_accel = _K_L1 * ground_speed * ground_speed / _L1_distance * sinf(eta);

  /* flying to waypoints, not circling them */
  //_circle_mode = false;

  /* the bearing angle, in NED frame */
  _bearing_error = eta;

};

};
