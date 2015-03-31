#pragma once

//user structs
#include "Planner/UserStructs/PlaneStateSim.h"
//#include "Planner/UserStructs/obstacle3D.h"
//#include "Planner/UserStructs/MissionSimFlagPt.h"
#include "nodes/UserStructs/GlobalPosi.h"
#include "nodes/UserStructs/PlaneAtt.h"
#include "nodes/UserStructs/GoalSetPt.h"
#include "nodes/UserStructs/AccelXYZ.h"

//ros msg header
#include "uascode/GlobalPos.h" //next to create the ros msgs
#include "uascode/PlaneAttitude.h"
//#include "uascode/MultiObsMsg.h"
//#include "uascode/IfRecMsg.h"
#include "uascode/AccelXYZ.h"
#include "uascode/WpCurrent.h"
//#include "uascode/IfMavlinkGood.h"
//#include "uascode/IfCollision.h"
#include "uascode/WpNumber.h"
//#include "uascode/ColliPoint.h"
#include "mavros/Mavlink.h"
#include "mavros/WaypointList.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/UInt16.h"

#include "ros/ros.h"

namespace UasCode{

class MavrosListen{

public:
    MavrosListen();
    ~MavrosListen();

    void working();

private:
    //plane state current
    UserStructs::PlaneStateSim st_current;
    //plane position
    UserStructs::GlobalPosi global_posi;
    //quaternion
    geometry_msgs::Quaternion plane_quat;
    //plane attitude
    UserStructs::PlaneAtt plane_att;
    //ACCEL
    UserStructs::AccelXYZ accel_xyz;
    //Current Waypoint
    int seq_current;

    //ros related
    ros::NodeHandle nh;
    //subscribers
    ros::Subscriber sub_posi;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_hdg;
    ros::Subscriber sub_att;
    ros::Subscriber sub_mc;//mission current
    ros::Subscriber sub_wps;//waypoints
    //service
    ros::ServiceClient client_wp;

    //callback functions
    void posiCb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void velCb(const std_msgs::Float64::ConstPtr& msg);
    void hdgCb(const std_msgs::Float64::ConstPtr& msg);
    void attCb(const sensor_msgs::Imu::ConstPtr& msg);
    void mission_currentCb(const std_msgs::UInt16::ConstPtr& msg);
    void wpsCb(const mavros::WaypointList::ConstPtr& msg);

};

}
