#pragma once
#include "armadillo"
#include <cmath>

namespace UasCode{
class L1ControlSim{
 public:
  L1ControlSim();
  
  void navigate_waypoints(const arma::vec::fixed<2> &pt_A,const arma::vec::fixed<2> &pt_B,const arma::vec::fixed<2> &curr_position,const arma::vec::fixed<2> &gnd_speed);
  void set_l1_period(float period);
  void set_l1_damping(float damping);
  void set_l1_roll_limit(float roll_lim_rad);
  float nav_roll();
  float nav_bearing();

 private:
  float _lateral_accel;		///< Lateral acceleration setpoint in m/s^2
  float _L1_distance;		///< L1 lead distance, defined by period and         damping
  //bool _circle_mode;		///< flag for loiter mode
  float _nav_bearing;		///< bearing to L1 reference point
  float _bearing_error;		///< bearing error
  float _crosstrack_error;	///< crosstrack error in meters
  float _target_bearing;		///< the heading setpoint

  float _L1_period;		///< L1 tracking period in seconds
  float _L1_damping;		///< L1 damping ratio
  float _L1_ratio;		///< L1 ratio for navigation
  float _K_L1;			///< L1 control gain for _L1_damping
  float _heading_omega;		///< Normalized frequency

  float _roll_lim_rad;  ///<maximum roll angle
//WGS1984 to plannar
  arma::vec::fixed<2> get_local_planar_vector(const arma::vec::fixed<2> &origin, const arma::vec::fixed<2> &target) const;

};

};
