#pragma once
#include "common/UserTypes/MavlinkSender.hpp"
#include "ros/ros.h"

//ros msg header
#include "uascode/PosSetPoint.h"
#include "uascode/IfRecMsg.h"
#include "uascode/IfCollision.h"

namespace UasCode {

class MavlinkSendNode
{
public:
  MavlinkSendNode();
  ~MavlinkSendNode();
  int PortSetUp();
  void working();
private:
  MavlinkSender sender;
  //ros related
  ros::NodeHandle nh;

  //subscriber
  ros::Subscriber sub_interwp;
  ros::Subscriber sub_IfRec;
  ros::Subscriber sub_IfColli;

  //callback functions
  void InterWpCb(const uascode::PosSetPoint::ConstPtr& msg);
  void IfRecCb(const uascode::IfRecMsg::ConstPtr& msg);
  void IfColliCb(const uascode::IfCollision::ConstPtr& msg);

  //contains
  double lat_s, lon_s, alt_s;
  bool if_receive;
  int if_colli;
  void SetDefault();
};

}



