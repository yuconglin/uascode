#include "MavlinkSendNode.hpp"
#include "ros/ros.h"

namespace UasCode{

MavlinkSendNode::MavlinkSendNode()
{
  sub_interwp= nh.subscribe("inter_wp",100,&MavlinkSendNode::InterWpCb,this);
  sub_IfRec= nh.subscribe("interwp_receive",100,&MavlinkSendNode::IfRecCb,this);
  sub_IfColli= nh.subscribe("if_colli",100,&MavlinkSendNode::IfColliCb,this);
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
     SetDefault();
     ros::spinOnce();

     if(!if_receive && lat_s!=0 && lon_s!=0 && alt_s!=0)
       sender.SendPosSP(lat_s,lon_s,alt_s);

     if(!if_colli== -1)
       sender.SendIfColli(if_colli);
  }
}

void MavlinkSendNode::InterWpCb(const uascode::PosSetPoint::ConstPtr& msg)
{
  lat_s = msg->lat;
  lon_s = msg->lon;
  alt_s = msg->alt;
}

void MavlinkSendNode::IfRecCb(const uascode::IfRecMsg::ConstPtr& msg)
{
  if_receive= msg->receive;
}

void MavlinkSendNode::IfColliCb(const uascode::IfCollision::ConstPtr& msg)
{
  if_colli= msg->if_collision;
}

void MavlinkSendNode::SetDefault()
{
   lat_s= 0.;
   lon_s= 0.;
   alt_s= 0.;
   if_colli = -1;
}

}
