#include "MavlinkSendNode.hpp"
#include "ros/ros.h"
#include "common/Utils/YcLogger.h"

namespace {
 Utils::LoggerPtr s_logger(Utils::getLogger("uascode.MavlinkSendNode.YcLogger") );
}

namespace UasCode{

MavlinkSendNode::MavlinkSendNode():wp_num(0),send_pos_method(0),lat_s(0.),lon_s(0.),alt_s(0.),seq_s(0),lat_c(0.),lon_c(0.),alt_c(0.),if_to_send(false)
{
  sub_interwp= nh.subscribe("inter_wp",100,&MavlinkSendNode::InterWpCb,this);
  sub_interwp_flag= nh.subscribe("inter_wp_flag",100,&MavlinkSendNode::InterWpFlagCb,this);
  sub_IfRec= nh.subscribe("interwp_receive",100,&MavlinkSendNode::IfRecCb,this);
  sub_IfColli= nh.subscribe("if_colli",100,&MavlinkSendNode::IfColliCb,this);
  sub_obss= nh.subscribe("multi_obstacles",100,&MavlinkSendNode::obssCb,this);
  sub_WpNum= nh.subscribe("wp_num",100,&MavlinkSendNode::WpNumCb,this);
  sub_colli_pt= nh.subscribe("colli_point",100,&MavlinkSendNode::ColliPtCb,this);
}

MavlinkSendNode::~MavlinkSendNode()
{

}

int MavlinkSendNode::PortSetUp()
{
  return sender.initialize();
}

void MavlinkSendNode::working()
{
  while(ros::ok())
  {
     //this->if_to_send = false;
     //SetDefault();
     ros::spinOnce();

     if(!if_receive)
     {
        /*
        UASLOG(s_logger,LL_DEBUG,"received wp: "
               << lat_s << " "
               << lon_s << " "
               << alt_s);
        */
        if(send_pos_method==0)
          sender.SendPosSP(lat_s,lon_s,alt_s);

        if(send_pos_method==1){
          UASLOG(s_logger,LL_DEBUG,"alt_s to send:"
                 << "lat_s:"<< lat_s << ","
                 << "lon_s:"<< lon_s << ","
                 << "alt_s:"<< alt_s);
          sender.SendPosSPflag(lat_s,lon_s,alt_s,seq_s,inter_exist);
        }
     }

     if(if_colli!= -1)
        sender.SendIfColli(if_colli);

     //sender.SendMultiObs(obss);
     sender.SendMultiObs3(obss);

     if(wp_num!=0)
       sender.SendWpNum(wp_num);

     sender.SendColliPt(lat_c,lon_c,alt_c);
  }

}

void MavlinkSendNode::InterWpCb(const uascode::PosSetPoint::ConstPtr& msg)
{
  lat_s = msg->lat;
  lon_s = msg->lon;
  alt_s = msg->alt;
}

void MavlinkSendNode::InterWpFlagCb(const uascode::PosSetPointFlag::ConstPtr &msg)
{
   lat_s = msg->lat;
   lon_s = msg->lon;
   alt_s = msg->alt;
   seq_s = msg->seq;
   inter_exist = msg->inter_exist;
   //this->if_to_send = true;
}

void MavlinkSendNode::IfRecCb(const uascode::IfRecMsg::ConstPtr& msg)
{
  if_receive= msg->receive;
}

void MavlinkSendNode::IfColliCb(const uascode::IfCollision::ConstPtr& msg)
{
  if_colli= msg->if_collision;
  UASLOG(s_logger,LL_DEBUG,"cb if_colli: "<< if_colli);
}

void MavlinkSendNode::obssCb(const uascode::MultiObsMsg::ConstPtr& msg)
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

    UASLOG(s_logger,LL_DEBUG,"obstacle callback: "
           << std::setprecision(4) << std::fixed
           << obs3d.t <<" "
           << obs3d.x1<<" "
           << obs3d.x2<<" "
           << obs3d.x3<<" ");

    obss.push_back(obs3d);
  }//for ends

}//obssCb ends

void MavlinkSendNode::WpNumCb(const uascode::WpNumber::ConstPtr &msg)
{
    wp_num= msg->wp_num;
    UASLOG(s_logger,LL_DEBUG,"waypoints number to send:"<< wp_num);
}

void MavlinkSendNode::ColliPtCb(const uascode::ColliPoint::ConstPtr &msg)
{
    UASLOG(s_logger,LL_DEBUG,"colli point received");
    lat_c = msg->lat;
    lon_c = msg->lon;
    alt_c = msg->alt;
}

void MavlinkSendNode::SetDefault()
{
   lat_s= 0.;
   lon_s= 0.;
   alt_s= 0.;
}

}
