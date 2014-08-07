#pragma once
#include "common/UserTypes/MavlinkSender.hpp"
#include "ros/ros.h"

#include "Planner/UserStructs/obstacle3D.h"

//ros msg header
#include "uascode/PosSetPoint.h"
#include "uascode/IfRecMsg.h"
#include "uascode/IfCollision.h"
#include "uascode/MultiObsMsg.h"

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
  ros::Subscriber sub_obss;

  //callback functions
  void InterWpCb(const uascode::PosSetPoint::ConstPtr& msg);
  void IfRecCb(const uascode::IfRecMsg::ConstPtr& msg);
  void IfColliCb(const uascode::IfCollision::ConstPtr& msg);
  void obssCb(const uascode::MultiObsMsg::ConstPtr& msg);

  //contains
  double lat_s, lon_s, alt_s;
  bool if_receive;
  int if_colli;
  std::vector<UserStructs::obstacle3D> obss;

  //private function
  void SetDefault();
};

}



