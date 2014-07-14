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
  spacelimit.LoadGeoFence("geofence.txt");
  path_gen.SetSpaceLimit(spacelimit); 
  path_gen.SetTimeLimit(1.0);
  path_gen.SetNinter(5);
  //if in ros?
  path_gen.SetInRos(false);
  //init state
  //double lat=33.388635, lon=-112.034193;
  double lat=33.440081, lon=-112.029698;
  double hgt= 1000;
  double spd= 25;
  //double t= Utils::GetTimeUTC();
  double t= 0;
  double yaw= 90./180*M_PI;
  double pitch= 0.;
  double alt_B= 1010;
  UserStructs::PlaneStateSim st(t,0,0,lat,lon,hgt,spd,yaw,pitch,0,0,0);
  st.GetUTM();

  
  std::cout<<"start_x: "<< st.x<<" "
           <<"start_y: "<< st.y<<" "
	   <<"start_z: "<< st.z<<" "
	   << std::endl;
  fs_points<<st.x<<" "<<st.y<<" "<<st.z<<" "<<std::endl;
  //goal waypoint
  //double lat1=33.374301,lon1=-111.842276;
  double lat1=33.437753,lon1= -111.985238;
  UserStructs::MissionSimPt pt(lat1,lon1,alt_B,yaw,100,0,0,200,250,150);
  pt.GetUTM();
  std::cout<<"goal_x: "<< pt.x<<" "
           <<"goal_y: "<< pt.y<<" "
	   <<"goal_z: "<< pt.alt<< std::endl;
  fs_points<<pt.x<<" "<<pt.y<<" "<<pt.alt<<std::endl;
  //set start state and goal waypoint
  path_gen.SetInitState(st);
  path_gen.SetGoalWp(pt);
  
  //parameters for the navigator
  //parameters
  double _Tmax= 12.49*UasCode::CONSTANT_G;
  double _Muav= 29.2; //kg
  double myaw_rate= 10./180*M_PI;
  double mpitch_rate= 5./180*M_PI;
  double _max_speed= 35; //m/s
  double _min_speed= 20; //m/s
  double _max_pitch= 20./180*M_PI;
  double _min_pitch= -20./180*M_PI;
  
  double dt= 1.;
  double _speed_trim= 30.;
  //set
  path_gen.NavUpdaterParams(_Tmax,mpitch_rate,myaw_rate,_Muav,_max_speed,_min_speed,_max_pitch,_min_pitch);

  path_gen.NavTecsReadParams("parameters.txt");
  path_gen.NavL1SetRollLim(10./180*M_PI);
  path_gen.NavSetDt(dt);
  path_gen.NavSetSpeedTrim(_speed_trim);
  //set sampler parameters
  path_gen.SetSampler(new UserTypes::SamplerPole() );
  path_gen.SetSampleParas();
  //set obstacles
  //start from one simple static obstacle 
  //obstacle3D(double _x1, double _x2, double _head_xy, double _speed, double _x3, double _v_vert, double _t, double _r, double _dr,double _hr,double _dhr): 
  double lat2=33.379461,lon2=-111.664435;
  double x_o,y_o;
  Utils::ToUTM(lon2,lat2,x_o,y_o);
  double head_xy= M_PI;
  //double spd= 35;
  double z_o = 1005;
  double vv= 0;
  double t1= t;
  double r= 640.08;
  double hr= 152.4;
  double dr=0, dhr=0;
  UserStructs::obstacle3D obs3d(x_o,y_o,head_xy,spd,z_o,vv,t1,r,dr,hr,dhr);
  std::vector<UserStructs::obstacle3D> v_obs3d;
  //v_obs3d.push_back(obs3d);
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

    /* 
    fs_two<< inter_wp.lat<<" "
          << inter_wp.lon<<" "
	  << inter_wp.alt<<" "
	  << inter_wp.yaw<<" "
	  << inter_wp.r<<" "
	  << inter_wp.x<<" "
	  << inter_wp.y<<" "
	  << inter_wp.h_rec<<" "
	  << inter_wp.v_rec<<" "
	  << inter_wp.alt_rec
	  << std::endl;*/
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
     
    //path_gen.PrintPath("/home/yucong/fuerte_workspace/sandbox/uascode/bin/path_log.txt");
    path_gen.PrintPath("path_log.txt");
  }
  std::cout<<"time used: "<< Utils::GetTimeNow()-tc1<< std::endl;
  return 0;
}
