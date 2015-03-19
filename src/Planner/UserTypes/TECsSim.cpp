#include "TECsSim.hpp"
#include "common/Utils/MathUtils.h"
#include "common/UserStructs/constants.h"
#include "common/Utils/GetTimeNow.h"
#include "common/Utils/YcLogger.h"
//std
#include <cmath>
#include <sstream>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>

namespace{
Utils::LoggerPtr s_logger(Utils::getLogger("uascode.TECsSim.YcLogger"));
}

namespace UasCode{
  TECsSim::TECsSim():
            _pitch_dem(0.0f),
            _integ1_state(0.0f),
	    _integ2_state(0.0f),
	    _integ3_state(0.0f),
	    _integ4_state(0.0f),
	    _integ5_state(0.0f),
	    _integ6_state(0.0f),
	    _integ7_state(0.0f),
	    _last_pitch_dem(0.0f),
	    _vel_dot(0.0f),
	    _TAS_dem(0.0f),
	    _TAS_dem_last(0.0f),
	    _hgt_dem_in_old(0.0f),
	    _hgt_dem_adj_last(0.0f),
	    _hgt_dem_prev(0.0f),
	    _TAS_dem_adj(0.0f),
	    _STEdotErrLast(0.0f),
	    _climbOutDem(false),
	    _SPE_dem(0.0f),
	    _SKE_dem(0.0f),
	    _SPEdot_dem(0.0f),
	    _SKEdot_dem(0.0f),
	    _SPE_est(0.0f),
	    _SKE_est(0.0f),
	    _SPEdot(0.0f),
	    _SKEdot(0.0f),
	    _airspeed_enabled(false),
	    _throttle_slewrate(100.0f)
  {
       //assign parameter name table
       name_map["FW_T_TIME_CONST"]= &_timeConst;
       name_map["FW_T_HGT_OMEGA"]= &_hgtCompFiltOmega;	
       name_map["FW_T_SPD_OMEGA"]= &_spdCompFiltOmega;	
       name_map["FW_T_CLMB_MAX"]= &_maxClimbRate;
       name_map["FW_T_SINK_MIN"]= &_minSinkRate;
       name_map["FW_T_SINK_MAX"]= &_maxSinkRate;
       name_map["FW_T_PTCH_DAMP"]= &_ptchDamp;
       name_map["FW_T_THR_DAMP"]= &_thrDamp;
       name_map["FW_T_INTEG_GAIN"]= &_integGain;
       name_map["FW_T_VERT_ACC"]= &_vertAccLim;	
       name_map["FW_T_RLL2THR"]= &_rollComp; 	
       name_map["FW_T_SPDWEIGHT"]=  &_spdWeight;
       name_map["FW_T_HRATE_P"]= &_heightrate_p;			
       name_map["FW_T_SRATE_P"]= &_speedrate_p;			
       name_map["FW_AIRSPD_MIN"]= &_indicated_airspeed_min;
       name_map["FW_AIRSPD_MAX"]= &_indicated_airspeed_max;
       name_map["FW_AIRSPD_TRIM"]= &_airspeed_trim;
       name_map["FW_THR_MIN"]= &throttle_min;
       name_map["FW_THR_MAX"]= &throttle_max;
       name_map["FW_THR_CRUISE"]= &throttle_cruise;
       t_inc= 0.;
       _update_50hz_last_sec= 0.;
       _update_speed_last_sec= 0.;
       _update_pitch_throttle_last_sec= 0.;
  }
  
  void TECsSim::update_50hz(float baro_altitude,const arma::vec::fixed<3> &accel_earth)
  {
    //std::cout<<"update_50hz"<< std::endl;
    //calculate time in secs since last update
    //std::cout<<"t_inc: "<< t_inc<< std::endl;
    double now= Utils::GetTimeNow();
    double DT= std::max((now- _update_50hz_last_sec),0.)+ t_inc;
    if(DT>1.0f){
      _integ3_state= baro_altitude;
      _integ2_state= 0.0f;
      _integ1_state= 0.0f;
      DT = 0.02;
    }

    _update_50hz_last_sec= now;

    _EAS= 0.5*(_indicated_airspeed_min+_indicated_airspeed_max);
    //get height acceleration
    //float hgt_ddot_mea= accel_earth(2)-CONSTANT_G;
    float hgt_ddot_mea= accel_earth(2);//here it is the total acceleration
    // Perform filter calculation using backwards Euler integration
    // Coefficients selected to place all three filter poles at omega
    float omega2 = _hgtCompFiltOmega * _hgtCompFiltOmega;
    //std::cout<<"baro_alt: "<< baro_altitude<< std::endl;
    float hgt_err = baro_altitude - _integ3_state;
    //std::cout<<"hgt_err: "<< hgt_err<< std::endl;
    float integ1_input = hgt_err * omega2 * _hgtCompFiltOmega;
    //std::cout<<"integ1_input: "<< integ1_input << std::endl;
    _integ1_state = _integ1_state + integ1_input * DT;
    //std::cout<<"_integ1_state: "<< _integ1_state<< std::endl;
    float integ2_input = _integ1_state + hgt_ddot_mea + hgt_err * omega2 * 3.0f;
    //std::cout<<"integ2_input: "<< integ2_input<< std::endl;
    _integ2_state = _integ2_state + integ2_input * DT;
    //std::cout<<"_integ2_state: "<< _integ2_state << std::endl;
    float integ3_input = _integ2_state + hgt_err * _hgtCompFiltOmega * 3.0f;
    //std::cout<<"integ3_input: "<< integ3_input<< std::endl;
    // If more than 1 second has elapsed since last update then reset the integrator state
    // to the measured height
    if (DT > 1.0f) {
	    _integ3_state = baro_altitude;
	    //if_start= true;

    } else {
	    _integ3_state = _integ3_state + integ3_input * DT;
    }
    //_vel_dot = 0.0f;
    _vel_dot = _TAS_rate_dem;
    //std::cout<<"_integ3_state:"<< _integ3_state<< std::endl;
  }

  void TECsSim::_update_speed(float airspeed_demand, float indicated_airspeed,
			 float indicated_airspeed_min, float indicated_airspeed_max, float EAS2TAS  )
  {
    // Calculate time in seconds since last update
    double now = Utils::GetTimeNow();
    float DT = std::max( (now - _update_speed_last_sec), 0.)+ t_inc;
    _update_speed_last_sec = now;

    _EAS_dem = airspeed_demand;
    _TAS_dem  = _EAS_dem * EAS2TAS;
    _TASmax   = indicated_airspeed_max * EAS2TAS;
    _TASmin   = indicated_airspeed_min * EAS2TAS;

    // Reset states of time since last update is too large
    if (DT > 1.0) {
	    _integ5_state = (_EAS * EAS2TAS);
	    _integ4_state = 0.0f;
	    DT            = 0.1; // when first starting TECS, use a
	    // small time constant
    }

    // Get airspeed or default to halfway between min and max if
    // airspeed is not being used and set speed rate to zero
    if (!airspeed_sensor_enabled()) {
	    // If no airspeed available use average of min and max
	    _EAS = 0.5f * (indicated_airspeed_min + indicated_airspeed_max);

    } else {
	    _EAS = indicated_airspeed;
    }

    // Implement a second order complementary filter to obtain a
    // smoothed airspeed estimate
    // airspeed estimate is held in _integ5_state
    float aspdErr = (_EAS * EAS2TAS) - _integ5_state;
    float integ4_input = aspdErr * _spdCompFiltOmega * _spdCompFiltOmega;

    // Prevent state from winding up
    if (_integ5_state < 3.1f) {
	    integ4_input = std::max(integ4_input , 0.0f);
    }

    _integ4_state = _integ4_state + integ4_input * DT;
    float integ5_input = _integ4_state + _vel_dot + aspdErr * _spdCompFiltOmega * 1.4142f;
    _integ5_state = _integ5_state + integ5_input * DT;
    // limit the airspeed to a minimum of 3 m/s
    _integ5_state = std::max(_integ5_state, 3.0f);
  }
 
  void TECsSim::_update_speed_demand(void)
  {
    if ((_badDescent) || (_underspeed)) {
      _TAS_dem     = _TASmin;
    }

    double velRateMax = 0.5*_STEdot_max / _integ5_state * 1.0;
    double velRateMin = 0.5*_STEdot_min / _integ5_state * 1.0;

    // Apply rate limit
    if ((_TAS_dem - _TAS_dem_adj) > (velRateMax * _DT))
    {
        _TAS_dem_adj = _TAS_dem_adj + velRateMax * _DT;
        _TAS_rate_dem = velRateMax;
    }
    else if ((_TAS_dem - _TAS_dem_adj) < (velRateMin * _DT))
    {
        _TAS_dem_adj = _TAS_dem_adj + velRateMin * _DT;
        _TAS_rate_dem = velRateMin;
    }
    else
    {
        _TAS_dem_adj = _TAS_dem;
        _TAS_rate_dem = (_TAS_dem - _TAS_dem_last) / _DT;
    } 
    //xxx: using a p loop for now
    // Constrain speed demand again to protect against bad values on initialisation.
    _TAS_dem_adj = Utils::math::constrain(_TAS_dem_adj, _TASmin, _TASmax);
    _TAS_dem_last = _TAS_dem;
  }

  void TECsSim::_update_height_demand(float demand,float state)
  {
    _hgt_dem_adj = demand;//0.025f * demand + 0.975f * _hgt_dem_adj_last;
    _hgt_rate_dem = (_hgt_dem_adj-state)*_heightrate_p;
    // Limit height rate of change
    if (_hgt_rate_dem > _maxClimbRate) {
	    _hgt_rate_dem = _maxClimbRate;

    } else if (_hgt_rate_dem < -_maxSinkRate) {
	    _hgt_rate_dem = -_maxSinkRate;
    } 
  }

  void TECsSim::_detect_underspeed(void)
  {
    if(((_integ5_state < _TASmin * 0.9f) && (_throttle_dem >= _THRmaxf * 0.95f)) || ((_integ3_state < _hgt_dem_adj) && _underspeed)) {
	    _underspeed = true;
    } else {
	    _underspeed = false;
    }
  }

  void TECsSim::_detect_bad_descent(void)
  {
    _badDescent= false;
  }

  void TECsSim::_update_energies(void)
  {

    // Calculate specific energy demands
    _SPE_dem = _hgt_dem_adj * CONSTANT_G;
    _SKE_dem = 0.5f * _TAS_dem_adj * _TAS_dem_adj;

    // Calculate specific energy rate demands
    _SPEdot_dem = _hgt_rate_dem * CONSTANT_G;
    _SKEdot_dem = _integ5_state * _TAS_rate_dem;

    // Calculate specific energy
    //std::cout<<"_integ3_state: "<< _integ3_state<< std::endl;
    _SPE_est = _integ3_state * CONSTANT_G;
    _SKE_est = 0.5f * _integ5_state * _integ5_state;

    // Calculate specific energy rate
    _SPEdot = _integ2_state * CONSTANT_G;
    _SKEdot = _integ5_state * _vel_dot;

  }
  
  void TECsSim::_update_throttle(float throttle_cruise,float yaw)
  {
    // Calculate total energy values
    _STE_error = _SPE_dem - _SPE_est + _SKE_dem - _SKE_est;
    float STEdot_dem = Utils::math::constrain((_SPEdot_dem + _SKEdot_dem), _STEdot_min, _STEdot_max);

    float STEdot_error = STEdot_dem - _SPEdot - _SKEdot;

    // Apply 0.5 second first order filter to STEdot_error
    // This is required to remove accelerometer noise from the  measurement
    STEdot_error = 0.2f * STEdot_error + 0.8f * _STEdotErrLast;
    _STEdotErrLast = STEdot_error;
    
    //Calculate throttle demand
    // Calculate gain scaler from specific energy error to throttle
    if(_underspeed) _throttle_dem=1.0f;
    else{
      float K_STE2Thr = 1 / (_timeConst * (_STEdot_max - _STEdot_min));

      // Calculate feed-forward throttle
      float ff_throttle = 0;
      float nomThr = throttle_cruise;
      // Use the demanded rate of change of total energy as the feed-forward demand, but add
      // additional component which scales with (1/cos(bank angle) - 1) to compensate for induced
      // drag increase during turns.
      //float cosPhi = sqrtf((rotMat(0, 1) * rotMat(0, 1)) + (rotMat(1, 1) * rotMat(1, 1)));
      STEdot_dem = STEdot_dem + _rollComp * (1.0f / Utils::math::constrain(fabs( cos(yaw) ) , 0.1f, 1.0f) - 1.0f);
      ff_throttle = nomThr + STEdot_dem / (_STEdot_max - _STEdot_min) * (_THRmaxf - _THRminf);
      // Calculate PD + FF throttle
      _throttle_dem = (_STE_error + STEdot_error * _thrDamp) * K_STE2Thr + ff_throttle;

      // Rate limit PD + FF throttle
      // Calculate the throttle increment from the specified slew time
      if (fabsf(_throttle_slewrate) > 0.01f) {
	      float thrRateIncr=_DT*(_THRmaxf-_THRminf)*_throttle_slewrate;

	      _throttle_dem = Utils::math::constrain(_throttle_dem,
					_last_throttle_dem - thrRateIncr,
					_last_throttle_dem + thrRateIncr);
	      _last_throttle_dem = _throttle_dem;
      }


      // Calculate integrator state upper and lower limits
      // Set to a value thqat will allow 0.1 (10%) throttle saturation to allow for noise on the demand
      float integ_max = (_THRmaxf - _throttle_dem + 0.1f);
      float integ_min = (_THRminf - _throttle_dem - 0.1f);

      // Calculate integrator state, constraining state
      // Set integrator to a max throttle value dduring climbout
      _integ6_state = _integ6_state + (_STE_error * _integGain)*_DT*K_STE2Thr;

      if (_climbOutDem) {
	      _integ6_state = integ_max;

      } else {
	      _integ6_state = Utils::math::constrain(_integ6_state, integ_min, integ_max);
      }

      // Sum the components.
      // Only use feed-forward component if airspeed is not being used
      if (airspeed_sensor_enabled()) {
	      _throttle_dem = _throttle_dem + _integ6_state;
              std::cout<<"airspeed_sensor_enabled"<< std::endl;
      } else {
	      _throttle_dem = ff_throttle;
      }
    
    }
    // Constrain throttle demand
    _throttle_dem = Utils::math::constrain(_throttle_dem, _THRminf, _THRmaxf);
  }

  void TECsSim::_update_pitch(void)
  {
    float SKE_weighting = Utils::math::constrain(_spdWeight, 0.0f, 2.0f);
    if ((_underspeed || _climbOutDem) && airspeed_sensor_enabled()) {
	    SKE_weighting = 2.0f;

    } else if (!airspeed_sensor_enabled()) {
	    SKE_weighting = 0.0f;
    }

    float SPE_weighting = 2.0f - SKE_weighting;
    // Calculate Specific Energy Balance demand, and error
    float SEB_dem      = _SPE_dem * SPE_weighting - _SKE_dem * SKE_weighting;
    float SEBdot_dem   = _SPEdot_dem * SPE_weighting - _SKEdot_dem * SKE_weighting;
    float SEB_error    = SEB_dem - (_SPE_est * SPE_weighting - _SKE_est * SKE_weighting);
    float SEBdot_error = SEBdot_dem - (_SPEdot * SPE_weighting - _SKEdot * SKE_weighting);
    float integ7_input = SEB_error * _integGain;

    if (_pitch_dem_unc > _PITCHmaxf) {
	    integ7_input = std::min(integ7_input, _PITCHmaxf - _pitch_dem_unc);

    } else if (_pitch_dem_unc < _PITCHminf) {
	    integ7_input = std::max(integ7_input, _PITCHminf - _pitch_dem_unc);
    }

    _integ7_state = _integ7_state + integ7_input * _DT;
    
    float gainInv = (_integ5_state * _timeConst * CONSTANT_G);

    float temp = SEB_error + SEBdot_error * _ptchDamp + SEBdot_dem * _timeConst;
    if (_climbOutDem)
    {
	    temp += _PITCHminf * gainInv;
    }

    _integ7_state = Utils::math::constrain(_integ7_state, (gainInv * (_PITCHminf - 0.0783f)) - temp, (gainInv * (_PITCHmaxf + 0.0783f)) - temp);

    // Calculate pitch demand from specific energy balance signals

    _pitch_dem_unc = (temp + _integ7_state) / gainInv;

    // Constrain pitch demand

    _pitch_dem = Utils::math::constrain(_pitch_dem_unc, _PITCHminf, _PITCHmaxf);
        // Rate limit the pitch demand to comply with specified vertical
    // acceleration limit

    float ptchRateIncr = _DT *_vertAccLim / _integ5_state;

    if ((_pitch_dem - _last_pitch_dem) > ptchRateIncr) {
	    _pitch_dem = _last_pitch_dem + ptchRateIncr;

    } else if ((_pitch_dem - _last_pitch_dem) < -ptchRateIncr) {
	    _pitch_dem = _last_pitch_dem - ptchRateIncr;
    }
    
    //std::cout<<"_pitch_dem: "<< _pitch_dem*180/M_PI << std::endl;

    _last_pitch_dem = _pitch_dem;
  }

 
  void TECsSim::_initialise_states(float pitch, float throttle_cruise, float baro_altitude, float ptchMinCO_rad)
  {
    // Initialise states and variables if DT > 1 second or in climbout
    if (_DT > 1.0f) {
	    _integ6_state      = 0.0f;
	    _integ7_state      = 0.0f;
	    _last_throttle_dem = throttle_cruise;
	    _last_pitch_dem    = pitch;
	    _hgt_dem_adj_last  = baro_altitude;
	    _hgt_dem_adj       = _hgt_dem_adj_last;
	    _hgt_dem_prev      = _hgt_dem_adj_last;
	    _hgt_dem_in_old    = _hgt_dem_adj_last;
	    _TAS_dem_last      = _TAS_dem;
	    _TAS_dem_adj       = _TAS_dem;
	    _underspeed        = false;
	    _badDescent        = false;
	    _DT                = 0.1f; // when first starting TECS, use a
	    // small time constant

    } else if (_climbOutDem) {
	    _PITCHminf          = ptchMinCO_rad;
	    _THRminf            = _THRmaxf - 0.01f;
	    _hgt_dem_adj_last  = baro_altitude;
	    _hgt_dem_adj       = _hgt_dem_adj_last;
	    _hgt_dem_prev      = _hgt_dem_adj_last;
	    _TAS_dem_last      = _TAS_dem;
	    _TAS_dem_adj       = _TAS_dem;
	    _underspeed        = false;
	    _badDescent        = false;
    }
 }

 void TECsSim::_update_STE_rate_lim(void)
 {
     // Calculate Specific Total Energy Rate Limits
     // This is a tivial calculation at the moment but will get bigger once we start adding altitude effects
     _STEdot_max = _maxClimbRate * CONSTANT_G;
     _STEdot_min = - _minSinkRate * CONSTANT_G;
 }

 //void TECsSim::update_pitch_throttle(float pitch, float baro_altitude, float hgt_dem, float EAS_dem, float indicated_airspeed, float EAS2TAS, bool climbOutDem, float ptchMinCO,
				// float throttle_min, float throttle_max, float throttle_cruise,
				// float pitch_limit_min, float pitch_limit_max)
void TECsSim::update_pitch_throttle(float pitch, float yaw,float baro_altitude, float hgt_dem, float EAS_dem, float indicated_airspeed, float EAS2TAS, bool climbOutDem)
 {
   // calculate time in sec since last update
   double now= Utils::GetTimeNow();

   _DT= std::max((now-_update_pitch_throttle_last_sec),0.)+ t_inc;

   _update_pitch_throttle_last_sec = now;

   // Update the speed estimate using a 2nd order complementary filter
   _update_speed(EAS_dem, indicated_airspeed, _indicated_airspeed_min, _indicated_airspeed_max, EAS2TAS);

   // Convert inputs
   _THRmaxf  = throttle_max;
   _THRminf  = throttle_min;
   _PITCHmaxf = pitch_limit_max;
   _PITCHminf = pitch_limit_min;
   _climbOutDem = climbOutDem;
   float ptchMinCO= pitch_limit_min;
   // initialise selected states and variables if DT > 1 second or in climbout
   //_initialise_states(pitch, throttle_cruise, baro_altitude, ptchMinCO);

   // Calculate Specific Total Energy Rate Limits
   _update_STE_rate_lim();

   // Calculate the speed demand
   _update_speed_demand();

   // Calculate the height demand
   _update_height_demand(hgt_dem, baro_altitude);

   // Detect underspeed condition
   _detect_underspeed();

   // Calculate specific energy quantitiues
   _update_energies();

   // Calculate throttle demand
   _update_throttle(throttle_cruise,yaw);

   // Detect bad descent due to demanded airspeed being too high
   _detect_bad_descent();

   // Calculate pitch demand
   _update_pitch();

 }

  void TECsSim::_load_param_from_file(const char *filename)
  {
    std::fstream myfile(filename); 
    if(myfile.is_open()){
        std::string line;
        while(std::getline(myfile,line))
        {
            std::istringstream iss(line);
            int c,comp;
            std::string name;
            float value;
            iss>>c >>comp >>name >>value;
            //find
            std::map<std::string,float*>::iterator it=name_map.find(name);
            if(it!=name_map.end())
            {
                *(it->second)= value;
            }

        }//while ends
    }
    else 
      std::cerr<<"error:parameter file unable to open."<<std::endl;
  }//_load_param_from_file ends


};
