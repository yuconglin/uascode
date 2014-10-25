#include "PlanNode3.hpp"
#include "common/UserStructs/constants.h"
#include "Planner/UserTypes/Sampler/SamplerPole.hpp"
#include "common/Utils/GetTimeUTC.h"
#include "common/Utils/GetTimeNow.h"
#include "common/Utils/YcLogger.h"
#include "common/Utils/UTMtransform.h"
#include "common/Utils/FindPath.h"
//mavros msg
#include "mavros/Mavlink.h"
//mavros service
#include "mavros/WaypointPush.h"
//std
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.PlanNode3.YcLogger"));
}

namespace UasCode{
//constructor
  PlanNode3::PlanNode3()
  {
    //publisher
    //pub_mavlink= nh.advertise<mavros::Mavlink>（"/mavlink/to"，100）;
    //subscriber
    //sub_mavlink= nh.subscribe("/mavlink/from",100,&PlanNode3::mavlinkCb,this);
    sub_obss= nh.subscribe("multi_obstacles",100,&PlanNode3::obssCb,this);
    sub_posi= nh.subscribe("/mavros/fix2",100,&PlanNode3::posiCb,this);
    sub_vel= nh.subscribe("/mavros/gps2_vel",100,&PlanNode3::velCb,this);
    sub_hdg= nh.subscribe("/mavros/gps2_hdg",100,&PlanNode3::hdgCb,this);
    sub_global_posi= nh.subscribe("/mavros/global_position/global",100,&PlanNode3::global_posiCb,this);
    sub_global_vel= nh.subscribe("/mavros/global_position/gps_vel",100,&PlanNode3::global_velCb,this);
    sub_global_hdg= nh.subscribe("/mavros/global_position/compass_hdg",100,&PlanNode3::global_hdgCb,this);
    sub_att= nh.subscribe("/mavros/imu/data",100,&PlanNode3::attCb,this);
    //service
    client_wp= nh.serviceClient<mavros::WaypointPush>("/mavros/mission/push");

    //parameters setting
    //parameters for controllers
    path_gen.SetTimeLimit(1.0);
    path_gen.SetNinter(5);

    //if in ros?
    path_gen.SetInRos(true);

    //set geofence/spacelimit
    UserStructs::SpaceLimit spacelimit(2000,500);
    //geofence.txt location needs changing.
    std::string fence_file = Utils::FindPath()+"parameters/geofence.txt";
    spacelimit.LoadGeoFence(fence_file.c_str());
    this->spLimit= spacelimit;
    path_gen.SetSpaceLimit(spacelimit);

    //set wp_r
    wp_r =30;

    thres_ratio=1.2;
    if_receive= false;
    //I use seq_current== -1 to indicate the moment mission starts
    seq_current= -1;
    seq_inter= 0;
    if_inter_gen= false;
    if_inter_exist= false;

    //parameters for the navigator
    double _Tmax= 12.49*UasCode::CONSTANT_G;
    double _Muav= 29.2; //kg
    double myaw_rate= 20./180*M_PI;
    double mpitch_rate= 10./180*M_PI;
    double _max_speed= 30; //m/s
    double _min_speed= 10; //m/s
    double _max_pitch= 25./180*M_PI;
    double _min_pitch= -20./180*M_PI;

    double dt= 1.;
    double _speed_trim= _max_speed;
    //set
    path_gen.NavUpdaterParams(_Tmax,mpitch_rate,myaw_rate,_Muav,_max_speed,_min_speed,_max_pitch,_min_pitch);

    std::string param_file = Utils::FindPath()+"parameters/parameters_sitl.txt";
    path_gen.NavTecsReadParams(param_file.c_str());
    path_gen.NavL1SetRollLim(40./180*M_PI);
    path_gen.NavSetDt(dt);
    path_gen.NavSetSpeedTrim(_speed_trim);

    //set sampler parameters
    path_gen.SetSampler(new UserTypes::SamplerPole() );
  }//constructor ends

  PlanNode3::~PlanNode3(){ }

  void PlanNode3::SetLogFileName(const char *filename)
  {
    try
    {
       traj_log.exceptions ( std::ofstream::failbit | std::ofstream::badbit );
       traj_log.open(filename,std::ofstream::out
                  | std::ofstream::in
                  | std::ofstream::trunc);
    }
    catch (std::ofstream::failure& e) {
          std::cerr << "Exception opening/reading file"
                    << e.what()
                    << std::endl;
    }

  }

  void PlanNode3::SetObsDisFile(const char *filename)
  {
      try
      {
          obdis_log.exceptions(std::ofstream::failbit | std::ofstream::badbit);
          obdis_log.open(filename,std::ofstream::out
                         | std::ofstream::in
                         | std::ofstream::trunc);
      }
      catch(std::ofstream::failure& e) {
          std::cerr << "Exception opening/reading file"
                    << e.what()
                    << std::endl;
      }
  }

  void PlanNode3::LoadFlightPlan(const char* filename)
  {
    std::ifstream plan_file(filename);
    int line_count= 0;
    //note altitude for each waypoint should be adjusted
    //by adding the height of home waypoint
    if(plan_file.is_open())
    {
       std::string line;
       while(getline(plan_file,line))
       {
         if(line_count == 0){
             std::string str1, str2, str3;
             std::istringstream iss(line);
             iss >> str1 >> str2 >> str3;
         }
         else{
               std::istringstream iss(line);
//0	1	0	16	0.00000  	0.000000	0.000000	0.000000	33.422036	-111.926263	30	1
               int seq,frame,command,current,autocontinue;
               float param1,param2,param3,param4;
               double lat,lon,alt;

               iss >> seq >> current >> frame >> command
                       >> param1 >> param2 >> param3 >> param4
                       >> lat >> lon >> alt >> autocontinue;

               alt= alt+home_alt;
               double r= 60;
               double x=0, y=0;
               Utils::ToUTM(lon,lat,x,y);
               double h= 200,v=150,alt_rec= 50;
               UASLOG(s_logger,LL_DEBUG,"push lat: "<< std::setprecision(6) << std::fixed
                      << lat<<" "<< alt);
               FlagWayPoints.push_back(UserStructs::MissionSimFlagPt(UserStructs::MissionSimPt(lat,lon,alt,0,r,x,y,h,v,alt_rec) ,false));

         }//if line_count > 0 ends
         ++line_count;
       }//while plan_file ends
       UASLOG(s_logger,LL_DEBUG,"loaded waypoints size: "<< FlagWayPoints.size() );
    }

    else{
      UASLOG(s_logger,LL_WARN," flight plan file cannot be loaded");
      return;
    }
  }

  void PlanNode3::SetTimeLimit(const double _t_limit)
  {
    t_limit= _t_limit;
    path_gen.SetTimeLimit(t_limit);
  }//SetTimeLimit ends

  int PlanNode3::PredictColliNode2(UserStructs::PlaneStateSim &st_current, int seq_current, double t_limit, double thres_ratio, UserStructs::PredictColliReturn &colli_return)
  {
      if(seq_current < 1) return -1;

      NavigatorSim* navigator_pt= path_gen.NavigatorPt();

      UASLOG(s_logger,LL_DEBUG,"predict colli starts");
      UASLOG(s_logger,LL_DEBUG,"FlagWayPoints size: " << FlagWayPoints.size() );

      std::ostringstream oss;
      for(int i=0;i!= FlagWayPoints.size();++i)
          oss << i<<":"<< FlagWayPoints[i].pt.lat <<" "
              << FlagWayPoints[i].pt.lon<<" "
              << FlagWayPoints[i].pt.alt <<" "
              << std::setprecision(7)<< std::fixed
              << FlagWayPoints[i].pt.x<< " "
              << FlagWayPoints[i].pt.y<< '\n';

      UASLOG(s_logger,LL_DEBUG,oss.str());

      bool tt= navigator_pt->PredictColli2(st_current,FlagWayPoints,wp_init,obss,spLimit,seq_current,t_limit,thres_ratio,colli_return);

      return (tt ? 1:0);
  }

  void PlanNode3::GetObssDis()
  {
      if(!obss.empty() )
      {
          std::ostringstream oss;
          oss<< "obss dis:";
          for(int i=0;i!= obss.size();++i)
          {
              double dis= std::sqrt(pow(st_current.x-obss[i].x1,2)
                                    +pow(st_current.y-obss[i].x2,2) );
              double dis_h= fabs(obss[i].x3-st_current.z);
              oss << " " << dis <<" "<< dis_h;
          }
          UASLOG(s_logger,LL_DEBUG,oss.str() );
      }
      if(!obss.empty() )
      {
          std::ostringstream oss;
          //oss<< "obss dis:";
          for(int i=0;i!= obss.size();++i)
          {
              double dis= std::sqrt(pow(st_current.x-obss[i].x1,2)
                                    +pow(st_current.y-obss[i].x2,2) );
              double dis_h= fabs(obss[i].x3-st_current.z);
              double dis_total= std::sqrt(dis*dis+dis_h*dis_h);
              oss << (int)obss[i].address <<" " << dis <<" "<< dis_h<<" "<< dis_total <<'\n';
          }
          obdis_log << oss.str();
      }
  }//GetObssDis() ends

  void PlanNode3::working()
  {
      possible_cases situ = NORMAL;
      ros::Rate r(10);
      while(ros::ok())
      {
         //callback once
         ros::spinOnce();

         r.sleep();
      }//while ends

  }//working() ends

  //callback functions
  void PlanNode3::obssCb(const uascode::MultiObsMsg::ConstPtr& msg)
  {
    if(!obss.empty() ) obss.clear();
    for(int i=0;i!= msg->MultiObs.size();++i)
    {
      UserStructs::obstacle3D obs3d(
        msg->MultiObs[i].address,
        msg->MultiObs[i].x1,
        msg->MultiObs[i].x2,
        msg->MultiObs[i].head_xy,
        msg->MultiObs[i].speed,
        msg->MultiObs[i].x3,
        msg->MultiObs[i].v_vert,
        msg->MultiObs[i].t,
        msg->MultiObs[i].r,0,
        msg->MultiObs[i].hr,0);

      UASLOG(s_logger,LL_DEBUG,"obstacle: "
             << std::setprecision(4) << std::fixed << msg->MultiObs[i].address<< " "
             << obs3d.t <<" "<< obs3d.x1);

      /*
      std::cout<< "obstacle: "<< std::setprecision(4) << std::fixed
               << obs3d.t << "\n";
      */
      obss.push_back(obs3d);
    }//for ends

  }//obssCb ends

  void PlanNode3::posiCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
     /*
     global_posi.lat= msg->latitude;
     global_posi.lon= msg->longitude;
     global_posi.alt= msg->altitude;
     */
      UASLOG(s_logger,LL_DEBUG,"posiCb:"
               << msg->latitude <<" "
               << msg->longitude<<" "
               << msg->altitude);

  }//posiCb ends

  void PlanNode3::velCb(const std_msgs::Float64::ConstPtr& msg)
  {
      UASLOG(s_logger,LL_DEBUG,"vecCb:"
             << msg->data
            );
  }//velCb ends

  void PlanNode3::hdgCb(const std_msgs::Float64::ConstPtr& msg)
  {
      UASLOG(s_logger,LL_DEBUG,"hdgCb:"<< msg->data*180./M_PI);
  }//hdgCb ends

  void PlanNode3::global_posiCb(const sensor_msgs::NavSatFix::ConstPtr &msg)
  {
      UASLOG(s_logger,LL_DEBUG,"global_posiCb:"
             << msg->latitude<<" "
             << msg->longitude<<" "
             << msg->altitude
             );
  }//global_posiCb ends

  void PlanNode3::global_velCb(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
  {
      double vel= std::sqrt(std::pow(msg->vector.x,2)
                            +std::pow(msg->vector.y,2)
                            +std::pow(msg->vector.z,2));
      UASLOG(s_logger,LL_DEBUG,"global_velCb:"
             << msg->vector.x <<" "
             << msg->vector.y <<" "
             << msg->vector.z <<" "
             << vel
             );
  }//global_velCb ends

  void PlanNode3::global_hdgCb(const std_msgs::Float64::ConstPtr &msg)
  {
      UASLOG(s_logger,LL_DEBUG,"global_hdgCb:"
             << msg->data
             );
  }//global_hdgCb ends

  void PlanNode3::attCb(const sensor_msgs::Imu::ConstPtr &msg)
  {
      UASLOG(s_logger,LL_DEBUG,"attCb:"
             << msg->orientation.x << " "
             << msg->orientation.y << " "
             << msg->orientation.z << " "
             << msg->orientation.w
             );
  }//attCb ends

}//namespace ends
