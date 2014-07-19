#pragma once

#include "common/UserTypes/MavlinkTCP.hpp"

//ros msgs
#include "uascode/GlobalPos.h"
#include "uascode/PlaneAttitude.h"
#include "uascode/IfRecMsg.h"
#include "uascode/AccelXYZ.h"

#include "ros/ros.h"

namespace UasCode {

class MavlinkRecNode
{
public:
   MavlinkRecNode();
   ~MavlinkRecNode();
   void TcpSetUp();
   void working();
private:
   MavlinkTCP mavlink_tcp;
   //ros related
   ros::NodeHandle nh;
   //publisher
   ros::Publisher pub_posi;
   ros::Publisher pub_att;
   ros::Publisher pub_IfRec;
   ros::Publisher pub_accel;
   //ros msg
   uascode::GlobalPos global_pos;
   uascode::PlaneAttitude plane_att;
   uascode::IfRecMsg if_rec;
   uascode::AccelXYZ accel_xyz;
};

}
