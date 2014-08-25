#include "PlanNode2.hpp"
#include "common/UserStructs/constants.h"
#include "Planner/UserTypes/Sampler/SamplerPole.hpp"
#include "common/Utils/GetTimeUTC.h"
#include "common/Utils/GetTimeNow.h"
#include "common/Utils/YcLogger.h"
#include "common/Utils/UTMtransform.h"

//std
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.PlanNode2.YcLogger"));
}

namespace UasCode{
//constructor
  PlanNode2::PlanNode2()
  {
    //publisher
    pub_interwp= nh.advertise<uascode::PosSetPoint>("inter_wp",100);
    pub_interwp_flag= nh.advertise<uascode::PosSetPointFlag>("inter_wp_flag",100);
    pub_if_colli= nh.advertise<uascode::IfCollision>("if_colli",100);
    pub_WpNum= nh.advertise<uascode::WpNumber>("wp_num",100);
    //subscriber
    sub_obss= nh.subscribe("multi_obstacles",100,&PlanNode2::obssCb,this);
    sub_pos= nh.subscribe("global_position",100,&PlanNode2::posCb,this);
    sub_att= nh.subscribe("plane_att",100,&PlanNode2::attCb,this);
    sub_IfRec= nh.subscribe("interwp_receive",100,&PlanNode2::ifRecCb,this);
    sub_accel= nh.subscribe("accel_raw_imu",100,&PlanNode2::AccelCb,this);
    sub_wp_current= nh.subscribe("waypoint_current",100,&PlanNode2::WpCurrCb,this);
    //sub_if_mavlink= nh.subscribe("if_mavlink",100,&PlanNode::IfMavlinkCb,this);

    //parameters for controllers
    path_gen.SetTimeLimit(1.0);
    path_gen.SetNinter(5);

    //if in ros?
    path_gen.SetInRos(true);

    //set geofence/spacelimit
    UserStructs::SpaceLimit spacelimit(2000,500);
    //geofence.txt location needs changing.
    spacelimit.LoadGeoFence("/home/yucong/ros_workspace/uascode/bin/geofence.txt");
    this->spLimit= spacelimit;
    path_gen.SetSpaceLimit(spacelimit);

    //set wp_r
    wp_r =30;

    thres_ratio=1.0;
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

    path_gen.NavTecsReadParams("/home/yucong/ros_workspace/uascode/parameters/parameters_sitl.txt");
    path_gen.NavL1SetRollLim(40./180*M_PI);
    path_gen.NavSetDt(dt);
    path_gen.NavSetSpeedTrim(_speed_trim);

    //set sampler parameters
    path_gen.SetSampler(new UserTypes::SamplerPole() );

  }//constructor ends

  void PlanNode2::SetLogFileName(const char *filename)
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

  void PlanNode2::SetObsDisFile(const char *filename)
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

  void PlanNode2::LoadFlightPlan(const char* filename)
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

  void PlanNode2::SetTimeLimit(const double _t_limit)
  {
    t_limit= _t_limit;
    path_gen.SetTimeLimit(t_limit);
  }//SetTimeLimit ends

  int PlanNode2::PredictColliNode(UserStructs::PlaneStateSim &st_current,int seq_current,double t_limit,double thres_ratio)
  {
    if(seq_current < 1) return -1;

    NavigatorSim* navigator_pt= path_gen.NavigatorPt();

    UASLOG(s_logger,LL_DEBUG,"predict colli starts");
    UASLOG(s_logger,LL_DEBUG,"FlagWayPoints size: " << FlagWayPoints.size() );

    std::ostringstream oss;
    for(int i=0;i!= FlagWayPoints.size();++i)
        oss << i<<":"<< FlagWayPoints[i].pt.lat <<" "
            << FlagWayPoints[i].pt.lon<<" "
            << FlagWayPoints[i].pt.alt << '\n';

    UASLOG(s_logger,LL_DEBUG,oss.str() );

    std::vector<UserStructs::MissionSimPt> waypoints;
    for(int i=0;i!=FlagWayPoints.size();++i)
        waypoints.push_back(FlagWayPoints[i].pt);

    bool tt= navigator_pt->PredictColli(st_current,waypoints,wp_init,obss,spLimit,seq_current,t_limit,thres_ratio);

    return (tt ? 1:0);
  }

  int PlanNode2::PredictColliNode2(UserStructs::PlaneStateSim &st_current, int seq_current, double t_limit, double thres_ratio, UserStructs::PredictColliReturn &colli_return)
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

  void PlanNode2::GetObssDis()
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
          oss<< "obss dis:";
          for(int i=0;i!= obss.size();++i)
          {
              double dis= std::sqrt(pow(st_current.x-obss[i].x1,2)
                                    +pow(st_current.y-obss[i].x2,2) );
              double dis_h= fabs(obss[i].x3-st_current.z);
              oss << (int)obss[i].address <<" " << dis <<" "<< dis_h <<'\n';
          }
          obdis_log << oss.str();
      }
  }

  void PlanNode2::predicting()
  {
     ros::Rate r(10);
     int seq_current_pre= 0;
     bool if_did= false;

     while(ros::ok())
     {
       ros::spinOnce();

       //to see if starts
       if(seq_current_pre < 1){
         if(seq_current==1)
         {
             wp_init.lat= global_posi.lat;
             wp_init.lon= global_posi.lon;
             wp_init.alt= global_posi.alt;
         }
         seq_current_pre= seq_current;
       }

       //check collision in 30 seconds
       GetCurrentSt();

       if(!if_did && seq_current > 0 && !obss.empty() )
       {
         bool if_colli= PredictColliNode(st_current,seq_current,1000,1.2);

         UASLOG(s_logger,LL_DEBUG,"if_colli: "
                << if_colli);

         if_did = true;
       }

     }

  }

  void PlanNode2::working()
  {
      possible_cases situ= NORMAL;
      int seq_current_pre= 0;
      ros::Rate r(10);

      while(ros::ok() )
      {
          //callback once
          ros::spinOnce();

          //to see if starts
          if(seq_current_pre < 1){

              if(seq_current== 1 )
              {
                  wp_init.lat= global_posi.lat;
                  wp_init.lon= global_posi.lon;
                  wp_init.alt= global_posi.alt;

                  UASLOG(s_logger,LL_DEBUG,"wp_init:" <<" "
                         << std::setprecision(6) << std::fixed
                         << "lat:" << wp_init.lat<< " "
                         << "lon:" << wp_init.lon<< " "
                         << "alt:" << wp_init.alt<< '\n');

              }
              seq_current_pre= seq_current;
          }

          //check collision in 30 seconds
          GetCurrentSt();
          GetObssDis();

          //int if_colli= PredictColliNode(st_current,seq_current,30,thres_ratio);
          int if_colli= PredictColliNode2(st_current,seq_current,30,thres_ratio,colli_return);

          UASLOG(s_logger,LL_DEBUG,"PredictColliNode: "<< if_colli);

          if(if_colli==1)
              UASLOG(s_logger,LL_DEBUG,"predict: "<< "seq:"<< colli_return.seq_colli<< " "
                     << "time:"<< colli_return.time_colli<<" "
                     << std::setprecision(7)<< std::fixed
                     << "x_colli:"<< colli_return.x_colli << " "
                     << "y_colli:"<< colli_return.y_colli << " "
                     << "z_colli:"<< colli_return.z_colli);

          IfColliMsg.if_collision = if_colli;
          pub_if_colli.publish(IfColliMsg);

          WpNumMsg.wp_num= FlagWayPoints.size();
          pub_WpNum.publish(WpNumMsg);

          if( if_colli == 1 && situ== NORMAL )
          {
              situ= PATH_GEN;
              if_inter_gen= false;
          }

          if(!if_inter_gen)
              thres_ratio= 1.5;
          else
              thres_ratio=1.;

          UASLOG(s_logger,LL_DEBUG,"situ="<< situ <<" "
                 << "if_receive="<<" "<< if_receive <<" "
                 << "seq_current="<<" "<< seq_current);

          //for diffrent cases
          switch(situ){

          case PATH_GEN:
          {
              //set start state and goal waypoint
              UASLOG(s_logger,LL_DEBUG,"planning");
              path_gen.SetInitState(st_current.SmallChange(t_limit));
              //get the start and goal for the sample
              int idx_end,idx_start=seq_current;//end and start of must go-through waypoint between current position and the goal
              this->seq_inter= colli_return.seq_colli;

              for(int i= colli_return.seq_colli;i!= FlagWayPoints.size();++i)
              {
                  if(!FlagWayPoints[i].flag){
                      path_gen.SetGoalWp(FlagWayPoints[i].pt);
                      idx_end= i-1;
                      break;
                  }
              }

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
              if (path_gen.AddPaths()> 0 )
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
                  set_pt.lat= inter_wp.lat;
                  set_pt.lon= inter_wp.lon;
                  //for mavproxy, home_alt must be subtracted
                  set_pt.alt= inter_wp.alt- home_alt;

                  double x_wp,y_wp;
                  Utils::ToUTM(set_pt.lon,set_pt.lat,x_wp,y_wp);

                  UASLOG(s_logger,LL_DEBUG,"wp generated: "
                         << set_pt.lat << " "
                         << set_pt.lon << " "
                         << x_wp << " "<< y_wp << " "
                         << set_pt.alt);

                  if(FlagWayPoints[seq_inter].flag){
                     if_inter_exist= true;
                     FlagWayPoints.erase(FlagWayPoints.begin()+seq_inter);
                  }
                  //this line is wrong
                  set_pt.seq= seq_inter;
                  set_pt.inter_exist= if_inter_exist ? 1:0;
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
              if(!if_receive){
                  //this will be sent to pixhawk by MavlinkReceiver node
                  UASLOG(s_logger,LL_DEBUG,"path ready for sending");
                  pub_interwp_flag.publish(set_pt);
              }
              else{
                  UASLOG(s_logger,LL_DEBUG,"path sent");
                  situ= NORMAL;
              }
              break;
          }

          case PATH_RECHECK:
          {
              if(path_gen.PathCheckSingle(st_current) )
                  situ= NORMAL;
              break;
          }

          default:
              break;
          }//switch ends

          r.sleep();
      }//while ends

  }//working

  //callback functions
  void PlanNode2::obssCb(const uascode::MultiObsMsg::ConstPtr& msg)
  {
    if(!obss.empty() ) obss.clear();
    for(int i=0;i!= msg->MultiObs.size();++i)
    {
      UserStructs::obstacle3D obs3d(
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
             << std::setprecision(4) << std::fixed
             << obs3d.t <<" "<< obs3d.x1);

      /*
      std::cout<< "obstacle: "<< std::setprecision(4) << std::fixed
               << obs3d.t << "\n";
      */
      obss.push_back(obs3d);
    }//for ends

  }//obssCb ends

  void PlanNode2::posCb(const uascode::GlobalPos::ConstPtr& msg)
  {
      global_posi.lat= msg->lat;
      global_posi.lon= msg->lon;
      global_posi.alt= msg->alt;
      global_posi.cog= msg->cog;
      global_posi.speed= msg->speed;
      /*
      std::cout<< "global_posi:"
               << global_posi.lat<< " "
               << global_posi.lon<< " "
               << global_posi.alt<< " "
               << global_posi.cog<< " "
               << global_posi.speed
               << std::endl;
      */
  }
  
  void PlanNode2::attCb(const uascode::PlaneAttitude::ConstPtr& msg)
  {
     plane_att.roll= msg->roll; 
     plane_att.pitch= msg->pitch;
     plane_att.yaw= msg->yaw;
     /*
     std::cout<< "plane_att:"
              << plane_att.roll<< " "
              << plane_att.pitch<< " "
              << plane_att.yaw
              << std::endl;
     */
  }

  void PlanNode2::ifRecCb(const uascode::IfRecMsg::ConstPtr &msg)
  {
     if_receive= msg->receive;

     //std::cout<< "if receive= "<< if_receive << std::endl;
  }

  void PlanNode2::AccelCb(const uascode::AccelXYZ::ConstPtr &msg)
  {
     accel_xyz.ax= msg->ax;
     accel_xyz.ay= msg->ay;
     accel_xyz.az= msg->az;
     /*
     std::cout<<"accel from raw imu: "
              << accel_xyz.ax <<" "
              << accel_xyz.ay <<" "
              << accel_xyz.az << std::endl;
     */
  }

  void PlanNode2::WpCurrCb(const uascode::WpCurrent::ConstPtr &msg)
  {
     seq_current= msg->wp_current;
     /*
     std::cout<<"current waypoint #: "
              << seq_current
              << std::endl;
     */
  }

  void PlanNode2::GetCurrentSt()
  {
    if(seq_current>0)
    {
        //st_current.t= Utils::GetTimeUTC();
        st_current.t= Utils::GetTimeNow();
        st_current.lat= global_posi.lat;
        st_current.lon= global_posi.lon;
        //get x,y
        st_current.GetUTM();
        st_current.z= global_posi.alt;
        st_current.speed= global_posi.speed;
        //st_current.yaw= plane_att.yaw;
        st_current.yaw= global_posi.cog*M_PI/180.;
        st_current.pitch= plane_att.pitch;

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

    }
  }

}//namespace ends
