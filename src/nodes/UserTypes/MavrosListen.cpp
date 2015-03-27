#include "MavrosListen.hpp"

//mavros msg
#include "mavros/Mavlink.h"
#include "mavros/Waypoint.h"
//mavros service
#include "mavros/WaypointPush.h"

#include "common/Utils/YcLogger.h"

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.MavrosListen.YcLogger"));
}

namespace UasCode{
  MavrosListen::MavrosListen()
  {
      sub_posi= nh.subscribe("/mavros/fix2",10,&MavrosListen::posiCb,this);
          sub_vel= nh.subscribe("/mavros/gps2_vel",10,&MavrosListen::velCb,this);
          sub_hdg= nh.subscribe("/mavros/gps2_hdg",10,&MavrosListen::hdgCb,this);
          sub_att= nh.subscribe("/mavros/imu/data",10,&MavrosListen::attCb,this);
          sub_mc= nh.subscribe("/mavros/mission_current",10,&MavrosListen::mission_currentCb,this);

  //service
  client_wp= nh.serviceClient<mavros::WaypointPush>("/mavros/mission/push");
  }

  MavrosListen::~MavrosListen(){
      //not any content yet
  }

  void MavrosListen::working()
  {
      ros::Rate r(10);
      while(ros::ok())
      {
         //callback once
         ros::spinOnce();
         r.sleep();
      }
  }

  void MavrosListen::posiCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
       global_posi.lat= msg->latitude;
       global_posi.lon= msg->longitude;
       global_posi.alt= msg->altitude;

       UASLOG(s_logger,LL_DEBUG,"posiCb:"
                 << msg->latitude <<" "
                 << msg->longitude<<" "
                 << msg->altitude);

    }//posiCb ends

    void MavrosListen::velCb(const std_msgs::Float64::ConstPtr& msg)
    {
        global_posi.speed = msg->data;

        UASLOG(s_logger,LL_DEBUG,"vecCb:"
               << msg->data
              );
    }//velCb ends

    void MavrosListen::hdgCb(const std_msgs::Float64::ConstPtr& msg)
    {
        global_posi.cog= msg->data;
        UASLOG(s_logger,LL_DEBUG,"hdgCb:"<< msg->data*180./M_PI);
    }//hdgCb ends

    void MavrosListen::attCb(const sensor_msgs::Imu::ConstPtr &msg)
    {
        plane_quat = msg->orientation;
        UASLOG(s_logger,LL_DEBUG,"attCb:"
               << msg->orientation.x << " "
               << msg->orientation.y << " "
               << msg->orientation.z << " "
               << msg->orientation.w
               );
    }//attCb ends

    void MavrosListen::mission_currentCb(const std_msgs::UInt16::ConstPtr &msg)
    {
        seq_current = (int)msg->data;
        UASLOG(s_logger,LL_DEBUG,"mission_currentCb:"
               << seq_current);
    }

}

