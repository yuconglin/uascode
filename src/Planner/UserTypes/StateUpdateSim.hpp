#pragma once
#include "Planner/UserStructs/PlaneStateSim.h"

namespace UasCode{
class StateUpdateSim{
  public:
   //construct
   StateUpdateSim();
   StateUpdateSim(double _Tmax,double _prate,double _yrate,double _M,
            double _max_spd,double _min_spd,double _max_pitch,double _min_pitch);
   void SetParams(double _Tmax,double _prate,double _yrate,double _M,
            double _max_spd,double _min_spd,double _max_pitch,double _min_pitch );
   UserStructs::PlaneStateSim update(UserStructs::PlaneStateSim &st_pre,
                        double nav_roll,
			double dem_yaw,
			double dem_pitch,
			double dem_thr,
                        double dt);  
   //set
   inline void SetTmax(double _Tmax){Tmax= _Tmax;}
   inline void SetMaxPitchRate(double _rate){mpitch_rate=_rate;}
   inline void SetMaxYawRate(double _rate){myaw_rate=_rate;}
   inline void SetMass(double _m){Muav= _m;}
   //get
   inline double GetMaxYawRate(){return this->myaw_rate;}
   inline double GetMaxPitch(){return max_pitch;}
   inline double GetMaxSpeed(){return max_speed;}
   //inline void SetVertAccLim(double _acc){_vertAccLim= _acc;}
   //void GetAccLim(double _hgt){ mpitch_rate= 

  private:
   double Tmax;
   double mpitch_rate;
   double myaw_rate;
   double Muav;//UAV's mass 
   //parameters readable
   //double _vertAccLim;
   //parameters limit
   double max_speed;
   double min_speed;
   double max_pitch;
   double min_pitch;
};

};
