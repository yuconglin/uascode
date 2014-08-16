#pragma once
//path generator
//input: start wp A, goal wp B, obstacles, geofence
//output: an intermediate wp P to avoid collision and stay within geofence
//constraints: time limit, GeoFence

#include "Planner/UserStructs/SpaceLimit.h"
#include "Planner/UserStructs/PlaneStateSim.h"
#include "Planner/UserStructs/obstacle3D.h"
#include "Planner/UserStructs/MissionSimPt.h"
#include "Planner/UserTypes/NavigatorSim.hpp"
#include "Planner/UserStructs/WpLength.h"
//ros
#include <ros/ros.h>

namespace UserTypes{
 class Sampler;
}

namespace UasCode{

class PathGenerator{
 public:
   PathGenerator();
   ~PathGenerator();
   //
   void SetSampler(UserTypes::Sampler* _sampler_pt);
   void SetSpaceLimit(UserStructs::SpaceLimit _limit);
   inline void SetTimeLimit(const double _t_lim){this->t_limit= _t_lim;}
   inline void SetInRos(bool _if_in){this->if_in_ros= _if_in;}
   inline void SetPlot(bool _if_plot){this->if_for_plot= _if_plot;}
   void SetObs(const std::vector<UserStructs::obstacle3D>& _obs3ds);
   //start state and goal wp
   void SetInitState(UserStructs::PlaneStateSim _st);
   void SetGoalWp(UserStructs::MissionSimPt& _pt);
   void SetInterState(UserStructs::PlaneStateSim& _st);
   void SetSampleStart(double _x_start,double _y_start,double _z_start);
   void SetBetweenWps(const std::vector<UserStructs::MissionSimPt> _wpoints);
   void SetSampleMethod(int _method);
   void SetNinter(const int _N);
   //set navigator parameters
   void NavUpdaterParams(double _Tmax,double _mpitch_rate,double _myaw_rate,double _Muav,
        double _max_speed,double _min_speed,double _max_pitch,double _min_pitch);
   void NavTecsReadParams(const char* filename);
   void NavL1SetRollLim(double _lim);
   void NavSetDt(double _dt);
   void NavSetSpeedTrim(double _trim);
   //about sampling
   void SetSampleParas();
   void SamplePt();
   //add paths
   int AddPaths();
   //clear paths
   void ClearToDefault();
   //generate the intermediate wp
   UserStructs::MissionSimPt GetInterWp();
   UserStructs::PlaneStateSim GetInterState();
   //path check with updated uav and obstacle states
   bool PathCheckSingle(UserStructs::PlaneStateSim _st);
   bool PathCheckRepeat(UserStructs::PlaneStateSim _st);
   void PrintPath(const char* filename);
   //return pointer to the navigator
   NavigatorSim* NavigatorPt(){return &(this->navigator);}

 private:
   //some technical parameter
   double max_yaw_rate;
   double max_pitch;
   double max_speed;
   double dt;
   //sample method
   int sample_method;
   //start state
   UserStructs::PlaneStateSim st_start;
   //goal waypoint
   UserStructs::MissionSimPt goal_wp;
   //sample waypoint
   UserStructs::MissionSimPt sample_wp;
   //final intermediate waypoint
   UserStructs::MissionSimPt inter_wp;
   //intermediate state when following the path
   UserStructs::PlaneStateSim st_inter;
   //the in-between waypoints we have to go through
   std::vector<UserStructs::MissionSimPt> WpInBetweens;
   //time limit for expanding paths in seconds
   double t_limit;
   ros::Time t_start;
   double sec_count;
   //record for all paths
   std::vector<UserStructs::WpLength> wp_lengths;
   //obstacles
   std::vector<UserStructs::obstacle3D> obs3ds;
   //GeoFence
   UserStructs::SpaceLimit spacelimit;
   //sampler
   UserTypes::Sampler* sampler_pt;
   //Navigator
   NavigatorSim navigator;
   //a vector of PlaneStateSim to store the temperary
   //plane state
   std::vector<UserStructs::PlaneStateSim> temp_rec;
   std::vector<UserStructs::StateNode> temp_part_rec;
   std::vector<UserStructs::PlaneStateSim> total_rec;
   //sample start
   double xs_start;
   double ys_start;
   double zs_start;
   //yaw_root
   double yaw_root;
   //flags to indicate sth was set or not
   bool if_start_set;
   bool if_goal_set;
   //bool if_goal_reach;
   bool if_sampler_set;
   bool if_sampler_para_set;
   bool if_spacelimit_set;
   //flags for options
   bool if_in_ros;
   bool if_for_plot;
   bool if_limit_reach;
   //check flags
   void CheckStartSet();
   void CheckGoalSet();
   void CheckSamplerSet();
   void CheckSamplerParaSet();
   void CheckSpaceLimitSet();
   //set yaw_root
   inline void SetYawRootSample(double _yaw_root){this->yaw_root= _yaw_root;}
};//class PathGenerator ends

};//namespace UasCode ends
