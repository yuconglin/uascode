#include "PlanNode.hpp"
#include "common/UserStructs/constants.h"
#include "Planner/UserTypes/Sampler/SamplerPole.hpp"
#include "common/Utils/GetTimeUTC.h"
#include "common/Utils/GetTimeNow.h"
#include "common/Utils/YcLogger.h"
#include "common/Utils/UTMtransform.h"
#include "common/Utils/FindPath.h"

//std
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.PlanNode.YcLogger"));
}

namespace UasCode{
//constructor
  PlanNode::PlanNode()
  {
    //publisher
    pub_interwp= nh.advertise<uascode::PosSetPoint>("inter_wp",100);
    pub_if_colli= nh.advertise<uascode::IfCollision>("if_colli",100);
    pub_WpNum= nh.advertise<uascode::WpNumber>("wp_num",100);
    //subscriber
    sub_obss= nh.subscribe("multi_obstacles",100,&PlanNode::obssCb,this);
    sub_pos= nh.subscribe("global_position",100,&PlanNode::posCb,this);
    sub_att= nh.subscribe("plane_att",100,&PlanNode::attCb,this);
    sub_IfRec= nh.subscribe("interwp_receive",100,&PlanNode::ifRecCb,this);
    sub_accel= nh.subscribe("accel_raw_imu",100,&PlanNode::AccelCb,this);
    sub_wp_current= nh.subscribe("waypoint_current",100,&PlanNode::WpCurrCb,this);

    //parameters for controllers
    path_gen.SetTimeLimit(1.0);
    path_gen.SetNinter(5);

    //if in ros?
    path_gen.SetInRos(true);

    //set geofence/spacelimit
    UserStructs::SpaceLimit spacelimit(2000,500);
    //geofence.txt location needs changing.
    std::string geofence_file= Utils::FindPath()+"parameters/geofence.txt";
    spacelimit.LoadGeoFence(geofence_file.c_str());
    this->spLimit= spacelimit;
    path_gen.SetSpaceLimit(spacelimit);

    //set wp_r
    wp_r =30;

    thres_ratio=1.0;
    if_receive= false;
    //I use seq_current== -1 to indicate the moment mission starts
    seq_current= -1;
    if_inter_gen= false;

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

    std::string params_file = Utils::FindPath()+"parameters/parameters_sitl.txt";
    path_gen.NavTecsReadParams(params_file.c_str());
    path_gen.NavL1SetRollLim(40./180*M_PI);
    path_gen.NavSetDt(dt);
    path_gen.NavSetSpeedTrim(_speed_trim);

    //set sampler parameters
    path_gen.SetSampler(new UserTypes::SamplerPole() );

  }//constructor ends

  void PlanNode::SetLogFileName(const char *filename)
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

  void PlanNode::LoadFlightPlan(const char* filename)
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
               waypoints.push_back(UserStructs::MissionSimPt(lat,lon,alt,0,r,x,y,h,v,alt_rec) );
               //waypoints.push_back( UserStructs::GoalSetPt(log[8],log[9],log[10]+home_alt) );
         }//if line_count > 0 ends
         ++line_count;
       }//while plan_file ends
       UASLOG(s_logger,LL_DEBUG,"loaded waypoints size: "<< waypoints.size() );
    }

    else{
      UASLOG(s_logger,LL_WARN," flight plan file cannot be loaded");
      return;
    }
  }

  void PlanNode::SetTimeLimit(const double _t_limit)
  {
    t_limit= _t_limit;
    path_gen.SetTimeLimit(t_limit);
  }//SetTimeLimit ends

  int PlanNode::PredictColliNode(UserStructs::PlaneStateSim &st_current,int seq_current,double t_limit,double thres_ratio)
  {
    //return true if collision predicted

    if(seq_current < 1) return -1;

    NavigatorSim* navigator_pt= path_gen.NavigatorPt();

    UASLOG(s_logger,LL_DEBUG,"predict colli starts");
    UASLOG(s_logger,LL_DEBUG,"waypoints size: " << waypoints.size() );

    std::ostringstream oss;
    for(int i=0;i!= waypoints.size();++i)
        oss << i<<":"<< waypoints[i].lat <<" "
            << waypoints[i].lon<<" "
            << waypoints[i].alt << '\n';

    UASLOG(s_logger,LL_DEBUG,oss.str() );

    bool tt= navigator_pt->PredictColli(st_current,waypoints,wp_init,obss,spLimit,seq_current,t_limit,thres_ratio);

    return (tt ? 1:0);
  }

  void PlanNode::GetObssDis()
  {
     if(!obss.empty() )
     {
         std::ostringstream oss;
         oss<< "obss dis:";
         for(int i=0;i!= obss.size();++i)
         {
             double dis= std::sqrt(pow(st_current.x-obss[i].x1,2)
                                   +pow(st_current.y-obss[i].x2,2)
                                   +pow(st_current.z-obss[i].x3,2));
             oss << " " << dis;
         }
         UASLOG(s_logger,LL_DEBUG,oss.str() );
     }
}

  void PlanNode::predicting()
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

  void PlanNode::working()
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

      int if_colli= PredictColliNode(st_current,seq_current,30,thres_ratio);
      UASLOG(s_logger,LL_DEBUG,"PredictColliNode: "<< if_colli);
      IfColliMsg.if_collision = if_colli;
      pub_if_colli.publish(IfColliMsg);

      WpNumMsg.wp_num= waypoints.size();
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
          path_gen.SetGoalWp(waypoints[seq_current]);
          path_gen.SetSampleParas();
          path_gen.SetObs(obss);
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

              set_pt.lat= inter_wp.lat;
              set_pt.lon= inter_wp.lon;
              //for mavproxy, home_alt must be subtracted
              set_pt.alt= inter_wp.alt- home_alt;

              UASLOG(s_logger,LL_DEBUG,"wp generated: "
                     << set_pt.lat << " "
                     << set_pt.lon << " "
                     << set_pt.alt);

              //here we need to add a flag to test if the waypoint is received

              if(inter_wp.lat!= waypoints[seq_current].lat || inter_wp.lon!= waypoints[seq_current].lon){
                 waypoints.insert(waypoints.begin()+seq_current,inter_wp);
                 situ= PATH_READY;
                 if_inter_gen= true;
              }
              else
                 situ= PATH_GEN;

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
              pub_interwp.publish(set_pt);
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
  void PlanNode::obssCb(const uascode::MultiObsMsg::ConstPtr& msg)
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
             << std::setprecision(4) << std::fixed
             << obs3d.t <<" "<< obs3d.x1);
      obss.push_back(obs3d);
    }//for ends

  }//obssCb ends

  void PlanNode::posCb(const uascode::GlobalPos::ConstPtr& msg)
  {
      global_posi.lat= msg->lat;
      global_posi.lon= msg->lon;
      global_posi.alt= msg->alt;
      global_posi.cog= msg->cog;
      global_posi.speed= msg->speed;
  }
  
  void PlanNode::attCb(const uascode::PlaneAttitude::ConstPtr& msg)
  {
     plane_att.roll= msg->roll; 
     plane_att.pitch= msg->pitch;
     plane_att.yaw= msg->yaw;
  }

  void PlanNode::ifRecCb(const uascode::IfRecMsg::ConstPtr &msg)
  {
     if_receive= msg->receive;

     //std::cout<< "if receive= "<< if_receive << std::endl;
  }

  void PlanNode::AccelCb(const uascode::AccelXYZ::ConstPtr &msg)
  {
     accel_xyz.ax= msg->ax;
     accel_xyz.ay= msg->ay;
     accel_xyz.az= msg->az;
  }

  void PlanNode::WpCurrCb(const uascode::WpCurrent::ConstPtr &msg)
  {
     seq_current= msg->wp_current;
  }

  void PlanNode::GetCurrentSt()
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
