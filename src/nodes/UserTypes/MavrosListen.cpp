#include "MavrosListen.hpp"

//mavros msg
#include "mavros/Mavlink.h"
#include "mavros/Waypoint.h"
#include "mavros/WaypointList.h"
//mavros service
#include "mavros/WaypointPush.h"
#include "mavros/WaypointPull.h"
#include <tf/LinearMath/Matrix3x3.h>

#include "common/Utils/YcLogger.h"
#include "common/Utils/GeoUtils.h"
#include "common/UserStructs/constants.h"
#include "Planner/UserTypes/Sampler/SamplerPole.hpp"
#include "common/Utils/GetTimeUTC.h"
#include "common/Utils/GetTimeNow.h"
#include "common/Utils/UTMtransform.h"
#include "common/Utils/FindPath.h"

//ros messages
#include "yucong_rosmsg/MultiObsMsg2.h"

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.MavrosListen.YcLogger"));
}

namespace UasCode{
  MavrosListen::MavrosListen():IfPullSent(false),PullSuccess(false),if_obss_update(false)
  {
          sub_posi= nh.subscribe("/mavros/global_position/global",10,&MavrosListen::posiCb,this);
          sub_posi_local= nh.subscribe("/mavros/global_position/local",10,&MavrosListen::posiLocalCb,this);
          sub_vel= nh.subscribe("/mavros/global_position/gp_vel",10,&MavrosListen::velCb,this);
          sub_local= nh.subscribe("/mavros/local_position/local",10,&MavrosListen::localCb,this);
          sub_att= nh.subscribe("/mavros/imu/data",10,&MavrosListen::attCb,this);
          sub_wps= nh.subscribe("/mavros/mission/waypoints",10,&MavrosListen::wpsCb,this);
          sub_wp_current= nh.subscribe("/mavros/mission_current",10,&MavrosListen::mission_currentCb,this);
          sub_state= nh.subscribe("/mavros/state",10,&MavrosListen::stateCb,this);
          sub_obss= nh.subscribe("/mavros/multi_obstacles",10,&MavrosListen::obssCb,this);

          //service
          client_wp_push = nh.serviceClient<mavros::WaypointPush>("/mavros/mission/push");
          client_wp_pull = nh.serviceClient<mavros::WaypointPull>("/mavros/mission/pull");

          wp_inter.frame= 0;
          wp_inter.command= 16;
          wp_inter.is_current= false;
          wp_inter.autocontinue= true;
          wp_inter.param1= 0;
          wp_inter.param2= 25;
          wp_inter.param3= -0.0;
          wp_inter.param4= 0;

          wp_r = 25;
          thres_ratio = 1.0;
          t_limit = 1.;

          seq_current= -1;
          seq_inter= 0;
          if_inter_gen= false;
          if_gen_success = false;
          if_inter_exist= false;
          if_obss_update= false;

          //parameters for the navigator
          double _Tmax= 6.79*CONSTANT_G;
          //double _Muav= 29.2; //kg
          double _Muav= 0.453592*13;
          double myaw_rate= 20./180*M_PI;
          double mpitch_rate= 10./180*M_PI;
          double _max_speed= 15; //m/s
          double _min_speed= 10; //m/s
          double _max_pitch= 25./180*M_PI;
          double _min_pitch= -20./180*M_PI;

          dt= 1.;
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

          //initialize helpers
          this->helpers = NULL;
  }

  MavrosListen::~MavrosListen(){
      //not any content yet
  }

  void MavrosListen::SetLogFileName(const char *filename)
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

  void MavrosListen::SetObsDisFile(const char *filename)
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

  void MavrosListen::working()
  {
      situ = NORMAL;
      ros::Rate r(10);

      while(ros::ok())
      {
         if_obss_update = false;
         //callback once
         ros::spinOnce();
         PrintSitu();

         //check collision in 30 seconds
         GetCurrentSt();
         GetObssDis();
         SetHelpers();

         if( waypoints.empty() )
         {
             WaypointsPull();
         }

         int if_colli = PredictColliNode3(st_current,seq_current,30,thres_ratio,colli_return);
         UASLOG(s_logger,LL_DEBUG,"PredictColliNode: "<< if_colli);
         UASLOG(s_logger,LL_DEBUG,"waypoints number: " << waypoints.size() );

         if(if_colli==1)
         {

             UASLOG(s_logger,LL_DEBUG,"predict: "<< "seq:"<< colli_return.seq_colli << " "
                    << "time:"<< colli_return.time_colli<<" "
                    << std::setprecision(7)<< std::fixed
                    << "x_colli:"<< colli_return.x_colli << " "
                    << "y_colli:"<< colli_return.y_colli << " "
                    << "z_colli:"<< colli_return.z_colli << " "
                    << "obstacle idx:"<< colli_return.obs_id);

             double w_x = st_current.x;
             double w_y = st_current.y;
             double w_z = st_current.z;

             double dis_c2d = std::sqrt(pow(w_x-colli_return.x_colli,2)+pow(w_y-colli_return.y_colli,2));
             double dis_cz = std::abs(w_z-colli_return.z_colli);
             UASLOG(s_logger,LL_DEBUG,"colli dis:"<< dis_c2d <<" "<< dis_cz);

             if(situ== NORMAL || situ== PATH_GEN )
             {
                 double c_lat, c_lon;
                 Utils::FromUTM(colli_return.x_colli,colli_return.y_colli,c_lon,c_lat);
                 colli_pt.lat = c_lat;
                 colli_pt.lon = c_lon;
                 UASLOG(s_logger,LL_DEBUG,"colli_point:"<< std::setprecision(6)<< std::fixed << colli_pt.lat <<" "<< colli_pt.lon);
                 colli_pt.alt = colli_return.z_colli;
             }

             //see if the colli point is too close to the sample root
             double rho= this->path_gen.GetTurnRadius();
             double obs_r= obss[0].r;
             double allow_dis = std::sqrt(pow(rho+obs_r,2)-pow(rho,2));
             UASLOG(s_logger,LL_DEBUG,
                    "allow_dis:" << allow_dis << ' '
                    << "rho:"<< rho << ' '
                    << "obs_r:"<< obs_r);

             if(seq_inter == seq_current-1 && if_gen_success)
             {
                 if_gen_success = false;
             }

             if(dis_c2d < allow_dis)
             {
                 if( !if_inter_gen && !if_gen_success )
                 {
                    UASLOG(s_logger,LL_DEBUG,"local avoidance");
                    for(int i = colli_return.seq_colli-1; i != 0; --i)
                    {
                        if(!flags[i]){
                            seq_inter = i + 1;
                            UASLOG(s_logger,LL_DEBUG,"seq_inter:" << seq_inter );
                            break;
                        }
                    }

                    if(colli_return.seq_colli == 0){
                        seq_inter = 0;
                    }

                    wp_inter.x_lat = colli_pt.lat;
                    wp_inter.y_long = colli_pt.lon;
                    wp_inter.z_alt = obss[colli_return.obs_id].x3 + obss[colli_return.obs_id].v_vert*colli_return.time_colli + 1.5*obss[colli_return.obs_id].hr;

                    if( flags[seq_inter] )
                    {
                        if_inter_exist= true;
                        waypoints.erase( waypoints.begin() + seq_inter );
                        flags.erase( flags.begin() + seq_inter );
                    }
                    else{
                        if_inter_exist= false;
                    }

                    waypoints.insert( waypoints.begin() + seq_inter, wp_inter );
                    flags.insert( flags.begin() + seq_inter, true );
                    if_inter_gen = true;

                    situ= PATH_READY;
                }
             }
             else
             {
                 UASLOG(s_logger,LL_DEBUG,"global avoidance");
                 if(situ== NORMAL)
                 {
                    situ= PATH_GEN;
                    if_inter_gen= false;
                 }
             }

         }//if_colli == 1 ends

         switch(situ)
         {
         case PATH_GEN:
         {
             UASLOG(s_logger,LL_DEBUG,"planning");
             if_gen_success = false;
             path_gen.SetInitState(st_current.SmallChange(t_limit));
             //get the start and goal for the sample
             int idx_end = 0;
             int idx_start = seq_current;//end and start of must go-through waypoint between current position and the goal

             //set path goal
             for(int i = colli_return.seq_colli; i != waypoints.size(); ++i )
             {
                 if( !flags[i] )
                 {
                     UserStructs::MissionSimPt pt;
                     MavrosWpToMissionPt( waypoints[i], pt );
                     path_gen.SetGoalWp( pt );
                     UASLOG(s_logger,LL_DEBUG,"flag i="<<" "<< i);
                     idx_end= i-1;
                     break;
                 }
             }//for ends

             //set path start
             for(int i = colli_return.seq_colli - 1; i != 0; --i)
             {
                 if( !flags[i] )
                 {
                     UserStructs::MissionSimPt pt;
                     MavrosWpToMissionPt( waypoints[i], pt );
                     path_gen.SetStartWp( pt );
                     seq_inter = i + 1;
                     break;
                 }
             }

             //set begin waypoint for navigation
             UserStructs::MissionSimPt pt;
             MavrosWpToMissionPt( waypoints[seq_current-1], pt );
             path_gen.SetBeginWp( pt );

             if(colli_return.seq_colli == seq_current)
             {
                path_gen.SetSampleStart(st_current.x, st_current.y, st_current.z);
             }

             if(colli_return.seq_colli == seq_current+1)
             {
                if( flags[seq_current] ){
                    path_gen.SetSampleStart(st_current.x, st_current.y, st_current.z);
                }
                else
                {
                    double x, y;
                    Utils::ToUTM(waypoints[seq_current].y_long, waypoints[seq_current].x_lat, x, y);
                    path_gen.SetSampleStart(x,y,waypoints[seq_current].z_alt);
                }
             }

             if(colli_return.seq_colli > seq_current + 1)
             {
                int c_id;
                if( flags[colli_return.seq_colli-1] )
                {
                    c_id = colli_return.seq_colli - 2;
                }
                else
                {
                    c_id = colli_return.seq_colli - 1;
                }
                double x, y;
                Utils::ToUTM(waypoints[c_id].y_long, waypoints[c_id].x_lat, x, y);
                path_gen.SetSampleStart(x,y,waypoints[c_id].z_alt);
             }

             path_gen.SetSampleParas();

             //get must go-through in-between waypoints
             std::vector< UserStructs::MissionSimPt > wpoints;
             UASLOG(s_logger,LL_DEBUG,"idx_start:"<< idx_start
                    << " "<<"idx_end:"<< idx_end);
             for(int i = idx_start; i < idx_end; ++i)
             {
                 UserStructs::MissionSimPt pt;
                 MavrosWpToMissionPt( waypoints[i], pt );
                 wpoints.push_back( pt );
             }
             path_gen.SetBetweenWps( wpoints );

             //to generate feasible paths
             if ( path_gen.AddPaths() > 0 )
                 situ= PATH_CHECK;
             else{
                 UASLOG(s_logger,LL_DEBUG,"no path, try again");
             }

             break;
         }
         case PATH_CHECK:
         {
             UserStructs::MissionSimPt inter_wp;
             UASLOG(s_logger,LL_DEBUG,"path check");
             if( path_gen.PathCheckRepeat(st_current) )
             {
                 UASLOG(s_logger,LL_DEBUG,"check ok, yes waypoint");
                 inter_wp = path_gen.GetInterWp();

                 if_inter_exist = false;
                 wp_inter.x_lat = inter_wp.lat;
                 wp_inter.y_long = inter_wp.lon;
                 wp_inter.z_alt = inter_wp.alt;

                 UASLOG(s_logger,LL_DEBUG,"wp generated:"
                        << inter_wp.lat << " "
                        << inter_wp.lon << " "
                        << inter_wp.alt);

                 if( flags[seq_inter] ){
                    if_inter_exist = true;
                    waypoints.erase( waypoints.begin() + seq_inter );
                    flags.erase( flags.begin() + seq_inter );
                    UASLOG(s_logger,LL_DEBUG,"inter wp exist");
                 }
                 else{
                    if_inter_exist = false;
                 }
                 UASLOG(s_logger,LL_DEBUG,"seq_inter:" << seq_inter);
                 waypoints.insert( waypoints.begin() + seq_inter, wp_inter);
                 flags.insert( flags.begin() + seq_inter, true );
                 if_inter_gen = true;
                 if_gen_success = true;
                 situ = PATH_READY;
             }
             else {
                 UASLOG(s_logger,LL_DEBUG,"No waypoint, retry");
                 situ= PATH_GEN;
             }

             break;
         }           
         case PATH_READY:
         {
             SendWaypoints();
             break;}
         default:
         {
             break;
         }
         }//switch(situ) ends

         //PullandSendWps();
         r.sleep();
      }//while ends

  }//working

  void MavrosListen::GetCurrentSt()
  {
      if( seq_current > 0 && if_posi_update && UavState == "AUTO.MISSION" )
      {
          st_current.t = Utils::GetTimeNow();
          st_current.lat= global_posi.lat;
          st_current.lon= global_posi.lon;
          //get x,y
          st_current.GetUTM();
          st_current.z= global_posi.alt;
          st_current.speed = global_posi.speed;
          st_current.yaw = global_posi.cog;
          st_current.pitch = plane_att.pitch;

          UASLOG(s_logger,LL_DEBUG,"st_current: "
                 << std::setprecision(4) << std::fixed
                 << st_current.t << " "
                 << st_current.x << " "
                 << st_current.y << " "
                 << st_current.z << " "
                 << st_current.speed << " "
                 << st_current.yaw * RAD2DEG <<" "
                 << st_current.pitch * RAD2DEG);

          if( traj_log.is_open() )
          {
              //UASLOG(s_logger,LL_DEBUG,"traj_log open");
              traj_log << std::setprecision(6) << std::fixed
                       << seq_current << " "
                       << st_current.t<< " "
                       << st_current.lat<< " "
                       << st_current.lon<< " "
                       << st_current.x<< " "
                       << st_current.y<< " "
                       << st_current.z<< " "
                       << st_current.speed<< " "
                       << st_current.yaw<< " "
                       << st_current.pitch << " "
                       << "\n";
          }

      }
  }

  bool MavrosListen::WaypointsPull()
  {
      PullSuccess = false;
      mavros::WaypointPull wp_pull_srv;
      client_wp_pull.call( wp_pull_srv );
      PullSuccess = wp_pull_srv.response.success;
      return PullSuccess;
  }

  void MavrosListen::PullandSendWps()
  {
      if( IfPullSent){
          return;
      }
      if( waypoints.empty() || !PullSuccess ){
          //call the waypoints pull service
          WaypointsPull();
      }
      else
      {
          //insert and push
          std::vector< mavros::Waypoint > wps = waypoints;
          for( int i = 0; i != wps.size(); ++i )
          {
              if( wps[i].is_current && i+2 < wps.size() )
              {
                  //wps[i].is_current = true;
                  //insert here
                  mavros::Waypoint mav_wp;
                  mav_wp.frame= 0;
                  mav_wp.command= 16;
                  mav_wp.is_current= false;
                  mav_wp.autocontinue= true;
                  mav_wp.param1= 0;
                  mav_wp.param2= 25;
                  mav_wp.param3= -0.0;
                  mav_wp.param4= 0;
                  mav_wp.x_lat= 0.5 * (wps[i].x_lat + wps[i+2].x_lat);
                  mav_wp.y_long= 0.5 * (wps[i].y_long + wps[i+2].y_long);
                  mav_wp.z_alt= 0.5 * (wps[i].z_alt + wps[i+2].z_alt);
                  wps.insert( wps.begin()+i+1, mav_wp );
              }
          }
          mavros::WaypointPush wp_push_srv;
          wp_push_srv.request.waypoints = wps;
          client_wp_push.call(wp_push_srv);
          if(wp_push_srv.response.success)
          {
              UASLOG(s_logger,LL_DEBUG,"new waypoints sent");
              IfPullSent = true;
          }
      }

  }

  void MavrosListen::PrintSitu()
  {
      switch(situ){
      case NORMAL:{
          UASLOG(s_logger,LL_DEBUG, "situ:NORMAL" << '\n');
          break;
      }
      case PATH_READY:{
          UASLOG(s_logger,LL_DEBUG, "situ: PATH_READY" << '\n');
          break;
      }
      case PATH_GEN:{
          UASLOG(s_logger,LL_DEBUG, "situ: PATH_GEN" << '\n');
          break;
      }
      case PATH_CHECK:{
          UASLOG(s_logger,LL_DEBUG, "situ: PATH_CHECK" << '\n');
          break;
      }
      default:
          break;
      }
  }

  void MavrosListen::GetObssDis()
  {
      /*
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

              if(dis < obss[i].r || dis_h < obss[i].hr )
                  if_fail = true;
          }
          UASLOG(s_logger,LL_DEBUG,oss.str() );
      }*/
      if(!obss.empty() )
      {
          std::ostringstream oss;
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
  }

  void MavrosListen::SetHelpers()
  {
      delete helpers;
      helpers = new std::vector< ObsHelper >();
      for(int i = 0; i!= obss.size(); ++i){
          if( if_obss_update ){
            obss[i].r *= thres_ratio;
          }
          helpers -> push_back(ObsHelper(obss[i],dt) );
      }
      //set helpers
      path_gen.NavSetHelpers(helpers);
      path_gen.NavSetIfSet( false );
  }

  void MavrosListen::MavrosWpToMissionPt(const mavros::Waypoint &wp, UserStructs::MissionSimPt& pt)
  {
      pt.lat = wp.x_lat;
      pt.lon = wp.y_long;
      pt.alt = wp.z_alt;
      pt.yaw = 0.0;
      pt.r = 25;
      pt.h_rec = 200;
      pt.v_rec = 150;
      pt.alt_rec = 10;
      pt.GetUTM();
  }

  void MavrosListen::SendWaypoints()
  {
      mavros::WaypointPush wp_push_srv;
      wp_push_srv.request.waypoints = waypoints;
      client_wp_push.call(wp_push_srv);
      if(wp_push_srv.response.success)
      {
          UASLOG(s_logger,LL_DEBUG,"new waypoints sent");
          situ = NORMAL;
          if_inter_gen = false;
      }
  }

  int MavrosListen::PredictColliNode3(UserStructs::PlaneStateSim &st_current,int seq_current,double t_limit,double thres_ratio,UserStructs::PredictColliReturn& colli_return)
  {
      if( seq_current < 0 ){
         return -1;
      }

      NavigatorSim* navigator_pt= path_gen.NavigatorPt();

      UASLOG(s_logger,LL_DEBUG,"predict colli starts");
      UASLOG(s_logger,LL_DEBUG,"WayPoints size: " << waypoints.size() );
      std::ostringstream oss;
      for(int i=0;i!= waypoints.size();++i)
          oss << i<<":"
              << std::setprecision(7)<< std::fixed
              << waypoints[i].x_lat <<" "
              << waypoints[i].y_long <<" "
              << waypoints[i].z_alt << '\n';

      UASLOG(s_logger,LL_DEBUG,oss.str());

      bool tt= navigator_pt->PredictColli3(st_current,waypoints,obss,spLimit,seq_current,t_limit,thres_ratio,colli_return);

      return (tt ? 1:0);
  }

  void MavrosListen::SetTimeLimit(const double _t_limit)
  {
      this->t_limit = _t_limit;
      path_gen.SetTimeLimit(t_limit);
  }//SetTimeLimit ends

  void MavrosListen::posiCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
       global_posi.lat= msg->latitude;
       global_posi.lon= msg->longitude;
       global_posi.alt= msg->altitude;
       if_posi_update = true;
       UASLOG(s_logger,LL_DEBUG,"global GCS:"
                 << msg->latitude <<" "
                 << msg->longitude<<" "
                 << msg->altitude);
       /*
       double x, y;
       Utils::ToUTM(lon,lat,x,y);
       UASLOG(s_logger,LL_DEBUG,"global UTM:"
              << x << " " << y);*/

    }//posiCb ends

    void MavrosListen::posiLocalCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        /*
        UASLOG(s_logger,LL_DEBUG,"global local posi:"
               << " " << x << " " << y << " " << z);
        */
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        tf::Matrix3x3 m( tf::Quaternion(qx,qy,qz,qw) );
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        UASLOG(s_logger,LL_DEBUG,"global local RPY:"
               << " " << roll * RAD2DEG
               << " " << pitch * RAD2DEG
               << " " << yaw * RAD2DEG);

        global_posi.cog= -yaw;
    }

    void MavrosListen::velCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        double vx = msg->vector.x;
        double vy = msg->vector.y;
        double vz = msg->vector.z;

        UASLOG(s_logger,LL_DEBUG,"vecCb:"
               << " " << vx << " " << vy << " " << vz
              );

        global_posi.speed = std::sqrt( vx*vx + vy*vy + vz*vz );
    }//velCb ends

    void MavrosListen::localCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        /*
        double local_x = msg->pose.position.x;
        double local_y = msg->pose.position.y;
        double local_z = msg->pose.position.z;

        UASLOG(s_logger,LL_DEBUG,"local position:"
               << " " << local_x
               << " " << local_y
               << " " << local_z);

        double qx = msg->pose.orientation.x;
        double qy = msg->pose.orientation.y;
        double qz = msg->pose.orientation.z;
        double qw = msg->pose.orientation.w;

        tf::Matrix3x3 m( tf::Quaternion(qx,qy,qz,qw) );
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        UASLOG(s_logger,LL_DEBUG,"local RPY:"
               << " " << roll * RAD2DEG
               << " " << pitch * RAD2DEG
               << " " << yaw * RAD2DEG);
               */
    }

    void MavrosListen::attCb(const sensor_msgs::Imu::ConstPtr &msg)
    {
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        tf::Matrix3x3 m( tf::Quaternion(qx,qy,qz,qw) );
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        UASLOG(s_logger,LL_DEBUG,"imu RPY:"
               << " " << roll * RAD2DEG
               << " " << pitch * RAD2DEG
               << " " << yaw * RAD2DEG);

        plane_att.roll = roll;
        plane_att.pitch = pitch;
        plane_att.yaw = -yaw;
    }//attCb ends

    void MavrosListen::wpsCb(const mavros::WaypointList::ConstPtr &msg )
    {
        waypoints = msg->waypoints;
        flags.clear();
        for( int i = 0; i != waypoints.size(); ++i )
        {
           UASLOG(s_logger,LL_DEBUG,"waypoints:" << " " << i << " "
                  << "is_current:" << (int)waypoints[i].is_current << " "
                  << "command:" << (int)waypoints[i].command << " "
                  << "autocontinue:" << (int)waypoints[i].autocontinue << " "
                  << waypoints[i].x_lat << " "
                  << waypoints[i].y_long << " "
                  << waypoints[i].z_alt);
           flags.push_back( false );
        }
    }//wpsCb

    void MavrosListen::mission_currentCb(const std_msgs::UInt16::ConstPtr &msg)
    {
        seq_current = (int)msg->data;
        //UASLOG(s_logger,LL_DEBUG,"mission_current:" << seq_current);
    }

    void MavrosListen::stateCb(const mavros::State::ConstPtr &msg)
    {
        UavState = msg->mode;
    }

    void MavrosListen::obssCb(const yucong_rosmsg::MultiObsMsg2::ConstPtr &msg)
    {
        obss.clear();
        for( int i = 0; i != msg->MultiObs.size(); ++i )
        {
            double lat = msg->MultiObs[i].lat;
            double lon = msg->MultiObs[i].lon;
            double x1, x2;
            Utils::ToUTM(lon,lat,x1,x2);

            UserStructs::obstacle3D obs3d(
                        msg->MultiObs[i].address,
                        x1,
                        x2,
                        msg->MultiObs[i].head_xy,
                        msg->MultiObs[i].speed,
                        msg->MultiObs[i].x3,
                        msg->MultiObs[i].v_vert,
                        msg->MultiObs[i].t,
                        msg->MultiObs[i].r,0,
                        msg->MultiObs[i].hr,0);

            UASLOG(s_logger,LL_DEBUG,"obstacle: "
                   << std::setprecision(4) << std::fixed << msg->MultiObs[i].address<< " "
                   << obs3d.t <<" "
                   << obs3d.x1 <<" "
                   << obs3d.x2 <<" "
                   << obs3d.x3 <<" "
                   << obs3d.speed <<" "
                   << obs3d.head_xy*180./M_PI <<" "
                   << obs3d.v_vert);

            obss.push_back(obs3d);
        }//for ends
        if_obss_update = true;
    }

}
