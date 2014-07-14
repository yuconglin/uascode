#pragma once
#include "Planner/UserStructs/PlaneStateSim.h"
#include "Planner/UserStructs/MissionSimPt.h"

namespace UserTypes{
  class Sampler{
	  
   public:
    Sampler();
    virtual ~Sampler(){};

    virtual void SetParams(UserStructs::PlaneStateSim& st,
	                   UserStructs::MissionSimPt& goal_wp,
			   double _sig_ga)=0;

    virtual void GetSample(double& x_a,double& y_a,double& z_a,
	                  UserStructs::PlaneStateSim& root_state,
			  UserStructs::MissionSimPt& goal_wp)=0;
    //set sampling method
    void SetSampleMethod(int _method);

   protected:
    double x0,y0,z0,ga0;//the start point
    double sigma_ga;//sigma for gamma angle
    int sample_method;
  };//abstract class Sampler ends

}//namespace UserTypes ends
