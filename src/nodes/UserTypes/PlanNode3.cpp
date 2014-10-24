#include "PlanNode3.hpp"
#include "common/UserStructs/constants.h"
#include "Planner/UserTypes/Sampler/SamplerPole.hpp"
#include "common/Utils/GetTimeUTC.h"
#include "common/Utils/GetTimeNow.h"
#include "common/Utils/YcLogger.h"
#include "common/Utils/UTMtransform.h"
#include "common/Utils/FindPath.h"
//mavros msg
#include "mavros/Mavlink.h"
//mavros service
#include "mavros/WaypointPull.h"
//std
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>

namespace {
    Utils::LoggerPtr s_logger(Utils::getLogger("uascode.PlanNode3.YcLogger"));
}

namespace UasCode{
//constructor
  PlanNode3::PlanNode3()
  {
    //publisher
    pub_mavlink= nh.advertise<mavros::Mavlink>（"/mavlink/to"，100）;
    //subscriber
    sub_obss= nh.subscribe("multi_obstacles",100,&PlanNode3::obssCb,this);
    sub_posi= nh.subscribe("~fix",100,&PlanNode3::posiCb,this);
    sub_vel= nh.subscribe("~gps_vel",100,&PlanNode3::velCb,this);
    sub_att= nh.subscribe("~imu/data",100,&PlanNode3::attCb,this);
    //service
    client_wp= nh.serviceClient<mavros::WaypointPush>("~mission/push");
  }


}
