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

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.MavrosListen.YcLogger"));
}

namespace UasCode{
  MavrosListen::MavrosListen():IfPullSent(false)
  {
          sub_posi= nh.subscribe("/mavros/global_position/global",10,&MavrosListen::posiCb,this);
          sub_posi_local= nh.subscribe("/mavros/global_position/local",10,&MavrosListen::posiLocalCb,this);
          sub_vel= nh.subscribe("/mavros/global_position/gp_vel",10,&MavrosListen::velCb,this);
          sub_local= nh.subscribe("/mavros/local_position/local",10,&MavrosListen::localCb,this);
          sub_att= nh.subscribe("/mavros/imu/data",10,&MavrosListen::attCb,this);
          sub_wps= nh.subscribe("/mavros/mission/waypoints",10,&MavrosListen::wpsCb,this);

  //service
  client_wp_push = nh.serviceClient<mavros::WaypointPush>("/mavros/mission/push");
  client_wp_pull = nh.serviceClient<mavros::WaypointPull>("/mavros/mission/pull");
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

  void MavrosListen::PullandSendWps()
  {
      if( IfPullSent){
          return;
      }



      IfPullSent = true;
  }

  void MavrosListen::posiCb(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
       double lat= msg->latitude;
       double lon= msg->longitude;
       double alt= msg->altitude;

       UASLOG(s_logger,LL_DEBUG,"global GCS:"
                 << msg->latitude <<" "
                 << msg->longitude<<" "
                 << msg->altitude);

       double x, y;
       Utils::ToUTM(lon,lat,x,y);
       UASLOG(s_logger,LL_DEBUG,"global UTM:"
              << x << " " << y);

    }//posiCb ends

    void MavrosListen::posiLocalCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;

        UASLOG(s_logger,LL_DEBUG,"global local posi:"
               << " " << x << " " << y << " " << z);

        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        tf::Matrix3x3 m( tf::Quaternion(qx,qy,qz,qw) );
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        UASLOG(s_logger,LL_DEBUG,"global local RPY:"
               << " " << roll
               << " " << pitch
               << " " << yaw);
    }

    void MavrosListen::velCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        double vx = msg->vector.x;
        double vy = msg->vector.y;
        double vz = msg->vector.z;
        UASLOG(s_logger,LL_DEBUG,"vecCb:"
               << " " << vx << " " << vy << " " << vz
              );
    }//velCb ends

    void MavrosListen::localCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
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
               << " " << roll
               << " " << pitch
               << " " << yaw);
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
               << " " << roll
               << " " << pitch
               << " " << yaw);
    }//attCb ends

    void MavrosListen::wpsCb(const mavros::WaypointList::ConstPtr &msg )
    {
        std::vector< mavros::Waypoint > waypoints;
        for( int i = 0; i != waypoints.size(); ++i )
        {
           UASLOG(s_logger,LL_DEBUG,"waypoints:" << " " << i << " "
                  << "is_current:" << waypoints[i].is_current << " "
                  << waypoints[i].x_lat << " "
                  << waypoints[i].y_long << " "
                  << waypoints[i].z_alt);
        }
     }//wpsCb

}
