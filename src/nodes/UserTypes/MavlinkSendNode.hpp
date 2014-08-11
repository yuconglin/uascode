#pragma once
#include "common/UserTypes/MavlinkSender.hpp"
#include "ros/ros.h"

#include "Planner/UserStructs/obstacle3D.h"

//ros msg header
#include "uascode/PosSetPoint.h"
#include "uascode/PosSetPointFlag.h"
#include "uascode/IfRecMsg.h"
#include "uascode/IfCollision.h"
#include "uascode/MultiObsMsg.h"
#include "uascode/WpNumber.h"

namespace UasCode {

class MavlinkSendNode
{
public:
  MavlinkSendNode();
  ~MavlinkSendNode();
  int PortSetUp();
  void working();
  inline void SetSendPosMethod(int _method){this->send_pos_method= _method;}
private:
  MavlinkSender sender;
  //ros related
  ros::NodeHandle nh;

  //subscriber
  ros::Subscriber sub_interwp;
  ros::Subscriber sub_interwp_flag;
  ros::Subscriber sub_IfRec;
  ros::Subscriber sub_IfColli;
  ros::Subscriber sub_obss;
  ros::Subscriber sub_WpNum;

  //callback functions
  void InterWpCb(const uascode::PosSetPoint::ConstPtr& msg);
  void InterWpFlagCb(const uascode::PosSetPointFlag::ConstPtr& msg);
  void IfRecCb(const uascode::IfRecMsg::ConstPtr& msg);
  void IfColliCb(const uascode::IfCollision::ConstPtr& msg);
  void obssCb(const uascode::MultiObsMsg::ConstPtr& msg);
  void WpNumCb(const uascode::WpNumber::ConstPtr& msg);

  //contains
  double lat_s, lon_s, alt_s;
  int inter_exist;
  bool if_receive;
  int if_colli;
  std::vector<UserStructs::obstacle3D> obss;
  int wp_num;

  int send_pos_method;

  //private function
  void SetDefault();
};

}



