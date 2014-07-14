#pragma once
#include "Sampler.hpp"
#include "Planner/UserStructs/PlaneStateSim.h"
#include "Planner/UserStructs/MissionSimPt.h"

namespace UserTypes{

   class SamplerPole:public Sampler{
    
    public:
      ~SamplerPole();

      void SetParams(UserStructs::PlaneStateSim& st, 
	             UserStructs::MissionSimPt& goal_wp,
		     double _sig_ga);
     void GetSample(double& x_a,double& y_a,double& z_a,
			UserStructs::PlaneStateSim& st,
			UserStructs::MissionSimPt& goal_wp);

    private:
      double r0;
      double sigma_r;
      double theta0;
     
   };//SamplerPole ends

}
