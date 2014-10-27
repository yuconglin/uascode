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
#include "mavros/Waypoint.h"
//mavros service
#include "mavros/WaypointPush.h"
//ros related
#include "tf/transform_datatypes.h"
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
    pub_colli_pt= nh.advertise<yucong_rosmsg::ColliPoint>("/mavros/colli_point",100);
    //subscriber

    sub_obss= nh.subscribe("/mavros/multi_obstacles",100,&PlanNode3::obssCb,this);
    sub_posi= nh.subscribe("/mavros/fix2",100,&PlanNode3::posiCb,this);
    sub_vel= nh.subscribe("/mavros/gps2_vel",100,&PlanNode3::velCb,this);
    sub_hdg= nh.subscribe("/mavros/gps2_hdg",100,&PlanNode3::hdgCb,this);
    sub_att= nh.subscribe("/mavros/imu/data",100,&PlanNode3::attCb,this);
    sub_mc= nh.subscribe("/mavros/mission_current",100,&PlanNode3::mission_currentCb,this);
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

  void PlanNode3::GetCurrentSt()
  {
      if(seq_current>0)
      {
          st_current.t= Utils::GetTimeNow();
          st_current.lat= global_posi.lat;
          st_current.lon= global_posi.lon;
          //get x,y
          st_current.GetUTM();
          st_current.z= global_posi.alt;
          st_current.speed= global_posi.speed;
          //st_current.yaw= plane_att.yaw;
          st_current.yaw= global_posi.cog*M_PI/180.;
          //quaternion to rpy
          tf::Quaternion quat;
          tf::quaternionMsgToTF(this->plane_quat, quat);
          // the tf::Quaternion has a method to acess roll pitch and yaw
          double roll, pitch, yaw;
          tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
          st_current.pitch= pitch;

          UASLOG(s_logger,LL_DEBUG,"st_current: "
                 << st_current.x << " "
                 << st_current.y << " "
                 << st_current.z << " "
                 << st_current.speed << " "
                 << st_current.yaw*180./M_PI <<" "
                 << global_posi.cog << " "
                 << st_current.pitch*180./M_PI);

          if(traj_log.is_open() ){
              traj_log << std::setprecision(6) << std::fixed
                       << st_current.t<< " "
                       << st_current.lat<< " "
                       << st_current.lon<< " "
                       << st_current.x<< " "
                       << st_current.y<< " "
                       << st_current.z<< " "
                       << st_current.speed<< " "
                       << st_current.yaw<< " "
                       << st_current.pitch
                       << "\n";
          }//
      }// if seq_current end
  }

  void PlanNode3:: SendWaypoints()
  {
      mavros::WaypointPush wp_push_srv;
      for(int i=0;i!= FlagWayPoints.size();++i)
      {
         mavros::Waypoint mav_wp;
         /*
uint8 frame
uint16 command
bool is_current
bool autocontinue
float32 param1
float32 param2
float32 param3
float32 param4
float64 x_lat
float64 y_long
float64 z_alt
          */
         mav_wp.frame= 0;
         mav_wp.command= 16;
         if(i== seq_current)
             mav_wp.is_current= true;
         else
             mav_wp.is_current= false;
         mav_wp.autocontinue= true;
         mav_wp.param1= 0;
         mav_wp.param2= 0;
         mav_wp.param3= 0;
         mav_wp.param4= 0;
         mav_wp.x_lat= FlagWayPoints[i].pt.lat;
         mav_wp.y_long= FlagWayPoints[i].pt.lon;
         mav_wp.z_alt= FlagWayPoints[i].pt.alt-home_alt;

         wp_push_srv.request.waypoints.push_back(mav_wp);

      }//for ends

      client_wp.call(wp_push_srv);
      if(wp_push_srv.response.success){
          UASLOG(s_logger,LL_DEBUG,"path sent");
          situ= NORMAL;
          if_inter_gen= false;
      }

  }//SendWaypoints ends

  void PlanNode3::working()
  {
      situ = NORMAL;
      int seq_current_pre = 0;
      ros::Rate r(10);
      while(ros::ok())
      {
         //callback once
         ros::spinOnce();

         //to see if starts
         if(seq_current_pre<1){

             if(seq_current== 1 )
             {
                 wp_init.lat= global_posi.lat;
                 wp_init.lon= global_posi.lon;
                 wp_init.alt= global_posi.alt;

                 FlagWayPoints[0].pt.lat = wp_init.lat;
                 FlagWayPoints[0].pt.lon = wp_init.lon;
                 FlagWayPoints[0].pt.alt = wp_init.alt;

                 UASLOG(s_logger,LL_DEBUG,"wp_init:" <<" "
                        << std::setprecision(6) << std::fixed
                        << "lat:" << wp_init.lat<< " "
                        << "lon:" << wp_init.lon<< " "
                        << "alt:" << wp_init.alt<< '\n');

             }

             seq_current_pre = seq_current;
         }

         GetCurrentSt();
         GetObssDis();

         int if_colli= PredictColliNode2(st_current,seq_current,30,thres_ratio,colli_return);

         UASLOG(s_logger,LL_DEBUG,"PredictColliNode: "<< if_colli);

         if(if_colli==1){
             UASLOG(s_logger,LL_DEBUG,"predict: "<< "seq:"<< colli_return.seq_colli<< " "
                    << "time:"<< colli_return.time_colli<<" "
                    << std::setprecision(7)<< std::fixed
                    << "x_colli:"<< colli_return.x_colli << " "
                    << "y_colli:"<< colli_return.y_colli << " "
                    << "z_colli:"<< colli_return.z_colli << " "
                    << "obstacle idx:"<< colli_return.obs_id);

             //get dis to imediate previous waypoint
             double w_x,w_y,w_z;
             if(colli_return.seq_colli>1)
             {
                 w_x = FlagWayPoints[colli_return.seq_colli-1].pt.x;
                 w_y = FlagWayPoints[colli_return.seq_colli-1].pt.y;
                 w_z = FlagWayPoints[colli_return.seq_colli-1].pt.alt;
             }
             else{
                 w_x = st_current.x;
                 w_y = st_current.y;
                 w_z = st_current.z;
             }

             double dis_c2d = std::sqrt(pow(w_x-colli_return.x_colli,2)+pow(w_y-colli_return.y_colli,2));
             double dis_cz = std::abs(w_z-colli_return.z_colli);
             UASLOG(s_logger,LL_DEBUG,"colli dis:"<< dis_c2d <<" "<< dis_cz);
             if(situ== NORMAL || situ== PATH_GEN){
                 double c_lat, c_lon;
                 Utils::FromUTM(colli_return.x_colli,colli_return.y_colli,c_lon,c_lat);
                 colli_pt.lat = c_lat;
                 colli_pt.lon = c_lon;
                 UASLOG(s_logger,LL_DEBUG,"colli_point:"<< std::setprecision(4)<< std::fixed << colli_pt.lat <<" "<< colli_pt.lon);
                 colli_pt.alt = colli_return.z_colli;
             }

             //see if the colli point is too close to the sample root
             double rho= this->path_gen.GetTurnRadius();
             double obs_r= obss[0].r;
             double allow_dis = std::sqrt(pow(rho+obs_r,2)-pow(rho,2));
             UASLOG(s_logger,LL_DEBUG,"allow_dis:" << allow_dis);
             if(dis_c2d < allow_dis && dis_c2d > st_current.speed*1.0) //using 0.5 to delay reaction and maitain height differenct
             {
                if(!if_inter_gen){
                    UASLOG(s_logger,LL_DEBUG,"local avoidance");
                    //UserStructs::MissionSimPt local_wp;
                    int insert_seq = colli_return.seq_colli-1;
                    if(colli_return.seq_colli==1)
                      insert_seq = 1;

                    double local_alt = obss[colli_return.obs_id].x3 + obss[colli_return.obs_id].v_vert*colli_return.time_colli + 1.5*obss[colli_return.obs_id].hr- home_alt;

                    if(FlagWayPoints[insert_seq].flag){
                        if_inter_exist= true;
                        FlagWayPoints.erase(FlagWayPoints.begin()+insert_seq);
                    }
                    else
                        if_inter_exist= false;

                    UserStructs::MissionSimPt local_wp = UserStructs::MissionSimPt(colli_pt.lat,colli_pt.lon,local_alt+home_alt,0,100,0,0,200,100,50);
                    local_wp.GetUTM();
                    FlagWayPoints.insert(FlagWayPoints.begin()+insert_seq,UserStructs::MissionSimFlagPt(local_wp,true) );
                    if_inter_gen = true;

                    if(situ== NORMAL || situ== PATH_GEN){
                        situ= PATH_READY;
                    }
                }
             }
             else{
                 UASLOG(s_logger,LL_DEBUG,"global avoidance");
                 if(situ== NORMAL){
                    situ= PATH_GEN;
                    if_inter_gen= false;
                 }
             }

         }
         pub_colli_pt.publish(colli_pt);

         UASLOG(s_logger,LL_DEBUG,"situ="<< situ <<" "
                << "if_receive="<<" "<< if_receive <<" "
                << "seq_current="<<" "<< seq_current);

         switch(situ){

         case PATH_GEN:
         {
             //set start state and goal waypoint
             UASLOG(s_logger,LL_DEBUG,"planning");
             path_gen.SetInitState(st_current.SmallChange(t_limit));
             //get the start and goal for the sample
             int idx_end,idx_start=seq_current;//end and start of must go-through waypoint between current position and the goal
             this->seq_inter= colli_return.seq_colli;

             //set path goal
             for(int i= colli_return.seq_colli;i!= FlagWayPoints.size();++i)
             {
                 if(!FlagWayPoints[i].flag){
                     path_gen.SetGoalWp(FlagWayPoints[i].pt);
                     idx_end= i-1;
                     break;
                 }
             }
             //set path start
             for(int i= colli_return.seq_colli-1;i!= 0;--i)
             {
                 if(!FlagWayPoints[i].flag){
                     path_gen.SetStartWp(FlagWayPoints[i].pt);
                     break;
                 }
             }
             //set begin waypoint for navigation
             path_gen.SetBeginWp(FlagWayPoints[seq_current-1].pt);

             if(colli_return.seq_colli == seq_current)
             {
                 path_gen.SetSampleStart(st_current.x,st_current.y,st_current.z);
             }

             if(colli_return.seq_colli == seq_current+1)
             {
                 if(FlagWayPoints[seq_current].flag){
                     path_gen.SetSampleStart(st_current.x,st_current.y,st_current.z);

                 }
                 else{
                     path_gen.SetSampleStart(FlagWayPoints[seq_current].pt.x,
                                             FlagWayPoints[seq_current].pt.y,
                                             FlagWayPoints[seq_current].pt.alt);
                 }
             }

             if(colli_return.seq_colli > seq_current+1)
             {
                 if(FlagWayPoints[colli_return.seq_colli-1].flag){
                     path_gen.SetSampleStart(FlagWayPoints[colli_return.seq_colli-2].pt.x,
                                             FlagWayPoints[colli_return.seq_colli-2].pt.y,
                                             FlagWayPoints[colli_return.seq_colli-2].pt.alt);
                 }
                 else{
                     path_gen.SetSampleStart(FlagWayPoints[colli_return.seq_colli-1].pt.x,
                                             FlagWayPoints[colli_return.seq_colli-1].pt.y,
                                             FlagWayPoints[colli_return.seq_colli-1].pt.alt);
                 }
             }

             path_gen.SetSampleParas();
             path_gen.SetObs(obss);

             //get must go-through in-between waypoints
             std::vector<UserStructs::MissionSimPt> wpoints;
             for(int i= idx_start;i<= idx_end;++i)
             {
                wpoints.push_back(FlagWayPoints[i].pt);
             }
             path_gen.SetBetweenWps(wpoints);

             //to generate feasible paths
             if (path_gen.AddPaths() > 0 )
                 situ= PATH_CHECK;
             else{
                 UASLOG(s_logger,LL_DEBUG,"no path, try again");
                 if(thres_ratio > 1.)
                     thres_ratio-= 0.1;
             }
             break;
         }

         case PATH_CHECK:
         {
             UserStructs::MissionSimPt inter_wp;
             UASLOG(s_logger,LL_DEBUG,"path check");
             if(path_gen.PathCheckRepeat(st_current))
             {
                 UASLOG(s_logger,LL_DEBUG,"check ok, yes waypoint");
                 inter_wp= path_gen.GetInterWp();

                 if_inter_exist= false;
                 double x_wp,y_wp;
                 Utils::ToUTM(inter_wp.lon,inter_wp.lat,x_wp,y_wp);

                 UASLOG(s_logger,LL_DEBUG,"wp generated: "
                        << inter_wp.lat << " "
                        << inter_wp.lon << " "
                        << x_wp << " "<< y_wp << " "
                        << inter_wp.alt);

                 if(FlagWayPoints[seq_inter].flag){
                    if_inter_exist= true;
                    FlagWayPoints.erase(FlagWayPoints.begin()+seq_inter);
                 }
                 else
                    if_inter_exist= false;


                 FlagWayPoints.insert(FlagWayPoints.begin()+seq_inter,UserStructs::MissionSimFlagPt(inter_wp,true) );
                 situ= PATH_READY;
                 if_inter_gen= true;
             }
             else{
                 UASLOG(s_logger,LL_DEBUG,"No waypoint, retry");
                 situ= PATH_GEN;
             }
             break;
         }

         case PATH_READY:
         {
             UASLOG(s_logger,LL_DEBUG,"path ready for sending");
             UASLOG(s_logger,LL_DEBUG,"if_inter_exist? "<< if_inter_exist);
             SendWaypoints();
         }

         default:
             break;

         }//switch ends

         r.sleep();
      }//while ends

  }//working() ends

  //callback functions
  void PlanNode3::obssCb(const yucong_rosmsg::MultiObsMsg2::ConstPtr& msg)
  {
    if(!obss.empty() ) obss.clear();
    for(int i=0;i!= msg->MultiObs.size();++i)
    {
      double x,y;
      Utils::ToUTM(msg->MultiObs[i].lon, msg->MultiObs[i].lat, x,y);
      UserStructs::obstacle3D obs3d(
        msg->MultiObs[i].address,
        x,
        y,
        msg->MultiObs[i].head_xy,
        msg->MultiObs[i].speed,
        msg->MultiObs[i].x3,
        msg->MultiObs[i].v_vert,
        msg->MultiObs[i].t,
        msg->MultiObs[i].r,0,
        msg->MultiObs[i].hr,0);

      UASLOG(s_logger,LL_DEBUG,"obstacle: "
             << std::setprecision(4) << std::fixed << msg->MultiObs[i].address<< " "
             << obs3d.t <<" "<< msg->MultiObs[i].lat <<" "<< msg->MultiObs[i].lon);

      /*
      std::cout<< "obstacle: "<< std::setprecision(4) << std::fixed
               << obs3d.t << "\n";
      */
      obss.push_back(obs3d);
    }//for ends

  }//obssCb ends

  void PlanNode3::posiCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {  
     global_posi.lat= msg->latitude;
     global_posi.lon= msg->longitude;
     global_posi.alt= msg->altitude;

     UASLOG(s_logger,LL_DEBUG,"posiCb:"
               << msg->latitude <<" "
               << msg->longitude<<" "
               << msg->altitude);

  }//posiCb ends

  void PlanNode3::velCb(const std_msgs::Float64::ConstPtr& msg)
  {
      global_posi.speed = msg->data;

      UASLOG(s_logger,LL_DEBUG,"vecCb:"
             << msg->data
            );
  }//velCb ends

  void PlanNode3::hdgCb(const std_msgs::Float64::ConstPtr& msg)
  {
      global_posi.cog= msg->data;
      UASLOG(s_logger,LL_DEBUG,"hdgCb:"<< msg->data*180./M_PI);
  }//hdgCb ends

  void PlanNode3::attCb(const sensor_msgs::Imu::ConstPtr &msg)
  {
      plane_quat = msg->orientation;
      UASLOG(s_logger,LL_DEBUG,"attCb:"
             << msg->orientation.x << " "
             << msg->orientation.y << " "
             << msg->orientation.z << " "
             << msg->orientation.w
             );
  }//attCb ends

  void PlanNode3::mission_currentCb(const std_msgs::UInt16::ConstPtr &msg)
  {
      seq_current = (int)msg->data;
      UASLOG(s_logger,LL_DEBUG,"mission_currentCb:"
             << seq_current);
  }

}//namespace ends
