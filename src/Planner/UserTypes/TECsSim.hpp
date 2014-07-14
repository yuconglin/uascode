#pragma once
#include "armadillo"
#include "string"
#include <map>

namespace UasCode{
  
  class TECsSim{
    public:
      //constructor
      TECsSim();
      //other
      bool airspeed_sensor_enabled() {
	      return _airspeed_enabled;
      }

      void enable_airspeed(bool enabled) {
	      _airspeed_enabled = enabled;
      } 

      void _load_param_from_file(const char* filename); 
      // Update of the estimated height and height rate internal state
      // Update of the inertial speed rate internal state
      // Should be called at 50Hz or greater
      // baro_altitude in meters
      void update_50hz(float baro_altitude,const arma::vec::fixed<3> &accel_earth);

      void update_1hz(float baro_altitude,const double accel_z);

      // Update the control loop calculations
      //void update_pitch_throttle(float pitch, float baro_altitude, float hgt_dem, float EAS_dem, float indicated_airspeed, float EAS2TAS, bool climbOutDem, float ptchMinCO,
	//			 float throttle_min, float throttle_max, float throttle_cruise,
	//			 float pitch_limit_min, float pitch_limit_max);
      void update_pitch_throttle(float pitch, float baro_altitude, float hgt_dem, float EAS_dem, float indicated_airspeed, float EAS2TAS, bool climbOutDem);

      // demanded throttle in percentage
      // should return 0 to 100
      float get_throttle_demand(void) {
	      return _throttle_dem;
      }
      int32_t get_throttle_demand_percent(void) {
	      return get_throttle_demand();
      }


      float get_pitch_demand() { return _pitch_dem; }

      // demanded pitch angle in centi-degrees
      // should return between -9000 to +9000
      int32_t get_pitch_demand_cd() { return int32_t(get_pitch_demand() * 5729.5781f);}

      // Rate of change of velocity along X body axis in m/s^2
      float get_VXdot(void) { return _vel_dot; }


      float get_speed_weight() {
	      return _spdWeight;
      }

      inline void set_time_const(float time_const) {
	      _timeConst = time_const;
      }

      inline void set_min_sink_rate(float rate) {
	      _minSinkRate = rate;
      }

      inline void set_max_sink_rate(float sink_rate) {
	      _maxSinkRate = sink_rate;
      }

      inline void set_max_climb_rate(float climb_rate) {
	      _maxClimbRate = climb_rate;
      }

      inline void set_throttle_damp(float throttle_damp) {
	      _thrDamp = throttle_damp;
      }

      inline void set_integrator_gain(float gain) {
	      _integGain = gain;
      }

      inline void set_vertical_accel_limit(float limit) {
	      _vertAccLim = limit;
      }

      inline void set_height_comp_filter_omega(float omega) {
	      _hgtCompFiltOmega = omega;
      }

      inline void set_speed_comp_filter_omega(float omega) {
	      _spdCompFiltOmega = omega;
      }

      inline void set_roll_throttle_compensation(float compensation) {
	      _rollComp = compensation;
      }

      inline void set_speed_weight(float weight) {
	      _spdWeight = weight;
      }

      inline void set_pitch_damping(float damping) {
	      _ptchDamp = damping;
      }

      inline void set_pitch_min(float _pitch_min) {
              pitch_limit_min= _pitch_min;
      }

      inline void set_pitch_max(float _pitch_max) {
              pitch_limit_max= _pitch_max;
      }

      inline void set_mpitch_rate(float _mpitch_rate) {
              mpitch_rate= _mpitch_rate;
      }

      inline void set_throttle_slewrate(float slewrate) {
	      _throttle_slewrate = slewrate;
      }

      inline void set_indicated_airspeed_min(float airspeed) {
	      _indicated_airspeed_min = airspeed;
      }

      inline void set_indicated_airspeed_max(float airspeed) {
	      _indicated_airspeed_max = airspeed;
      }

      inline void set_heightrate_p(float heightrate_p) {
	      _heightrate_p = heightrate_p;
      }

      inline void set_speedrate_p(float speedrate_p) {
	      _speedrate_p = speedrate_p;
      }
      
      //inline void set_dt(float _dt){dt= _dt;}
      inline void SetTinc(double _dt){t_inc= _dt;}
    private:
      //strcut for load parameters
      std::map<std::string,float*> name_map;
      //time between two updates
      double t_inc;
      //Last time update 50Hz was called
      double _update_50hz_last_sec;
      
      //last time update_speed was called
      double _update_speed_last_sec;
      
      //last time update_pitch_throttle was called
      double _update_pitch_throttle_last_sec;

      // TECS tuning parameters
      float _hgtCompFiltOmega;
      float _spdCompFiltOmega;
      float _maxClimbRate;
      float _minSinkRate;
      float _maxSinkRate;
      float _timeConst;
      float _ptchDamp;
      float _thrDamp;
      float _integGain;
      float _vertAccLim;
      float _rollComp;
      float _spdWeight;
      float _heightrate_p;
      float _speedrate_p;
      //float _airspeed_min;
      //float _airspeed_max;
      float _airspeed_trim;

      // throttle demand in the range from 0.0 to 1.0
      float _throttle_dem;

      // pitch angle demand in radians
      float _pitch_dem;

      // Integrator state 1 - height filter second derivative
      float _integ1_state;

      // Integrator state 2 - height rate
      float _integ2_state;

      // Integrator state 3 - height
      float _integ3_state;

      // Integrator state 4 - airspeed filter first derivative
      float _integ4_state;

      // Integrator state 5 - true airspeed
      float _integ5_state;

      // Integrator state 6 - throttle integrator
      float _integ6_state;

      // Integrator state 6 - pitch integrator
      float _integ7_state;

      // throttle demand rate limiter state
      float _last_throttle_dem;

      // pitch demand rate limiter state
      float _last_pitch_dem;

      // Rate of change of speed along X axis
      float _vel_dot;

      // Equivalent airspeed
      float _EAS;

      // True airspeed limits
      float _TASmax;
      float _TASmin;

      // Current and last true airspeed demand
      float _TAS_dem;
      float _TAS_dem_last;

      // Equivalent airspeed demand
      float _EAS_dem;

      // height demands
      float _hgt_dem;
      float _hgt_dem_in_old;
      float _hgt_dem_adj;
      float _hgt_dem_adj_last;
      float _hgt_rate_dem;
      float _hgt_dem_prev;

      // Speed demand after application of rate limiting
      // This is the demand tracked by the TECS control loops
      float _TAS_dem_adj;

      // Speed rate demand after application of rate limiting
      // This is the demand tracked by the TECS control loops
      float _TAS_rate_dem;

      // Total energy rate filter state
      float _STEdotErrLast;

      // Underspeed condition
      bool _underspeed;

      // Bad descent condition caused by unachievable airspeed demand
      bool _badDescent;

      // climbout mode
      bool _climbOutDem;

      // throttle demand before limiting
      float _throttle_dem_unc;

      // pitch demand before limiting
      float _pitch_dem_unc;

      // Maximum and minimum specific total energy rate limits
      float _STEdot_max;
      float _STEdot_min;

      // Maximum and minimum floating point throttle limits
      float _THRmaxf;
      float _THRminf;

      // Maximum and minimum floating point pitch limits
      float _PITCHmaxf;
      float _PITCHminf;

      // Specific energy quantities
      float _SPE_dem;
      float _SKE_dem;
      float _SPEdot_dem;
      float _SKEdot_dem;
      float _SPE_est;
      float _SKE_est;
      float _SPEdot;
      float _SKEdot;

      //for update_throttle_pitch
      float throttle_min;
      float throttle_max;
      float throttle_cruise;
      float pitch_limit_min;
      float pitch_limit_max;
      float airspeed_trim;
      // Specific energy error quantities
      float _STE_error;

      // added by Yucong Lin
      float mpitch_rate;
      // Time since last update of main TECS loop (seconds)
      float _DT;

      bool _airspeed_enabled;
      float _throttle_slewrate;
      float _indicated_airspeed_min;
      float _indicated_airspeed_max;

      // Update the airspeed internal state using a second order complementary filter
      void _update_speed(float airspeed_demand, float indicated_airspeed,
			 float indicated_airspeed_min, float indicated_airspeed_max, float EAS2TAS);

      // Update the demanded airspeed
      void _update_speed_demand(void);

      // Update the demanded height
      void _update_height_demand(float demand, float state);

      // Detect an underspeed condition
      void _detect_underspeed(void);

      // Update Specific Energy Quantities
      void _update_energies(void);

      // Update Demanded Throttle
      void _update_throttle(float throttle_cruise);

      // Detect Bad Descent
      void _detect_bad_descent(void);

      // Update Demanded Pitch Angle
      void _update_pitch(void);

      // Initialise states and variables
      void _initialise_states(float pitch, float throttle_cruise, float baro_altitude, float ptchMinCO_rad);

      // Calculate specific total energy rate limits
      void _update_STE_rate_lim(void);

  };

};
