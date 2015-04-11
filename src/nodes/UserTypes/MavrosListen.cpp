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

          wp_r = 25;
          thres_ratio = 1.0;

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
         if_posi_update = false;
         if_obss_update = false;
         //callback once
         ros::spinOnce();
         PrintSitu();

         //check collision in 30 seconds
         GetCurrentSt();
         GetObssDis();
         SetHelpers();

         int if_colli = PredictColliNode2(st_current,seq_current,30,thres_ratio,colli_return);
         UASLOG(s_logger,LL_DEBUG,"PredictColliNode: "<< if_colli);


         //PullandSendWps();
         r.sleep();
      }
  }

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
        for( int i = 0; i != waypoints.size(); ++i )
        {
           UASLOG(s_logger,LL_DEBUG,"waypoints:" << " " << i << " "
                  << "is_current:" << (int)waypoints[i].is_current << " "
                  << "command:" << (int)waypoints[i].command << " "
                  << "autocontinue:" << (int)waypoints[i].autocontinue << " "
                  << waypoints[i].x_lat << " "
                  << waypoints[i].y_long << " "
                  << waypoints[i].z_alt);
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
