//test for PathGenerator.cpp
#include "PathGenerator.hpp"
//USR TYPES
#include "Planner/UserStructs/SpaceLimit.h"
#include "Planner/UserStructs/PlaneStateSim.h"
#include "Planner/UserStructs/MissionSimPt.h"
#include "Planner/UserTypes/Sampler/SamplerPole.hpp"
#include "common/UserStructs/constants.h"
//utils
#include "common/Utils/UTMtransform.h"
#include "common/Utils/GetTimeNow.h"
#include "common/Utils/FindPath.h"
//std
#include <iostream>
#include <fstream>

int main(int argc, char** argv)
{
  std::ofstream fs_points("out_points.txt");
  std::ofstream fs_two("out_two.txt");
  //start a path generator
  UasCode::PathGenerator path_gen;
  //spacelimit
  UserStructs::SpaceLimit spacelimit(2000,500);
  spacelimit.LoadGeoFence( (Utils::FindPath()+"parameters/geofence.txt").c_str() );
  path_gen.SetSpaceLimit(spacelimit); 
  path_gen.SetTimeLimit(1.0);
  //path_gen.SetNinter(5);
  //if in ros?
  path_gen.SetInRos(false);
  //init state

  //414722 3.69845e+06 613.31 30.35 272.58 272.58 4.58893

  double xs = 414722;
  double ys = 3.69845e+06;
  double hgt= 613.31;
  double spd= 30.35;
  //double t= Utils::GetTimeUTC();
  double t= 0;
  double yaw= (90.-272.58)/180*M_PI;
  double pitch= 4.58893/180*M_PI;

  UserStructs::PlaneStateSim st(t,xs,ys,0,0,hgt,spd,yaw,pitch,0,0,0);
  st.GetGCS();

  std::cout<<"start_x: "<< st.x<<" "
           <<"start_y: "<< st.y<<" "
	   <<"start_z: "<< st.z<<" "
	   << std::endl;
  fs_points<<st.x<<" "<<st.y<<" "<<st.z<<" "<<std::endl;

  //start waypoint
  double lat1=33.4219620, lon1= -111.9090470, alt_A = 615;
  UserStructs::MissionSimPt pt(lat1,lon1,alt_A,0,100,0,0,200,250,150);
  pt.GetUTM();

  //set start state and goal waypoint
  path_gen.SetInitState(st);
  path_gen.SetStartWp(pt);

  //goal waypoint
  double lat2=33.4219620, lon2= -111.9399030, alt_B = 615.;
  UserStructs::MissionSimPt pt1(lat2,lon2,alt_B,0,100,0,0,200,250,150);
  pt1.GetUTM();
  path_gen.SetGoalWp(pt1);

  path_gen.SetSampleStart(st.x,st.y,st.z);

  //parameters for the navigator
  //parameters
  double _Tmax= 6.79*UasCode::CONSTANT_G;
  double _Muav= 0.453592*13;
  double myaw_rate= 20./180*M_PI;
  double mpitch_rate= 10./180*M_PI;
  double _max_speed= 30.8667; //m/s
  double _min_speed= 10; //m/s
  double _max_pitch= 25./180*M_PI;
  double _min_pitch= -20./180*M_PI;
  
  double dt= 1.;
  double _speed_trim= _max_speed;
  //set
  path_gen.NavUpdaterParams(_Tmax,mpitch_rate,myaw_rate,_Muav,_max_speed,_min_speed,_max_pitch,_min_pitch);

  path_gen.NavTecsReadParams((Utils::FindPath()+"parameters/parameters_sitl.txt").c_str());
  path_gen.NavL1SetRollLim(10./180*M_PI);
  path_gen.NavSetDt(dt);
  path_gen.NavSetSpeedTrim(_speed_trim);
  //set sampler parameters
  path_gen.SetSampler(new UserTypes::SamplerPole() );
  path_gen.SetSampleParas();

  //set obstacles
  //414866.5938 3698033.0000 522.8900 114.7211 144.8438 18.2067
  double xo= 414866.5938,yo= 3698033.0000,zo= 522.8900;

  double head_xy= 144.8438/180.*M_PI;
  double spd1= 114.7211;
  double vv= 18.2067;
  double t1= t;
  double r= 300;
  double hr= 50;
  double dr=0, dhr=0;
  UserStructs::obstacle3D obs3d(11,xo,yo,head_xy,spd1,zo,vv,t1,r,dr,hr,dhr);
  std::vector<UserStructs::obstacle3D> v_obs3d;
  v_obs3d.push_back(obs3d);
  path_gen.SetObs(v_obs3d);

  double tc1= Utils::GetTimeNow();
  //add paths
  std::cout<< "num inter wps: "
           <<path_gen.AddPaths()<< std::endl;
  //st_current
  UserStructs::PlaneStateSim st_current= st;
  st_current.x= st.x;
  st_current.y= st.y;
  st_current.z= st.z;
  st_current.GetGCS();
  //
  UserStructs::MissionSimPt inter_wp;
  if(path_gen.PathCheckRepeat(st_current)){
    std::cout<<"check ok" << std::endl;
    inter_wp= path_gen.GetInterWp();
    std::cout<<"inter_x: "<< inter_wp.x<<" "
             <<"inter_y: "<< inter_wp.y<<" "
	     <<"inter_z: "<< inter_wp.alt<< std::endl;
    
    fs_points<<inter_wp.x<<" "<<inter_wp.y<<" "
      <<inter_wp.alt<< std::endl;
    
    UserStructs::PlaneStateSim st_inter= path_gen.GetInterState();
    fs_two<< st_inter.t<<" "
          << st_inter.x<<" "
	  << std::setprecision(10)<< st_inter.y<<" "
	  << st_inter.lat<<" "
	  << st_inter.lon<<" "
	  << st_inter.z<<" "
	  << st_inter.speed<<" "
	  << st_inter.yaw<<" "
	  << st_inter.pitch<<" "
	  << st_inter.ax<<" "
	  << st_inter.ay<<" "
	  << st_inter.az<<" "
	  << std::endl;

    fs_two<< pt.lat<<" "
          << pt.lon<<" "
          << pt.alt<<" "
          << pt.yaw<<" "
          << pt.r<<" "
	  << pt.x<<" "
	  << pt.y<<" "
	  << pt.h_rec<<" "
	  << pt.v_rec<<" "
	  << pt.alt_rec
	  << std::endl;
     
    path_gen.PrintPath("path_log.txt");
  }
  std::cout<<"time used: "<< Utils::GetTimeNow()-tc1<< std::endl;
  return 0;
}
