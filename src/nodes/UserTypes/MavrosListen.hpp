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
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

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
    //geometry_msgs::Quaternion plane_quat;
    //plane attitude
    UserStructs::PlaneAtt plane_att;
    //ACCEL
    UserStructs::AccelXYZ accel_xyz;
    //Current Waypoint
    int seq_current;
    //if pulled and sent
    bool IfPullSent;
    bool PullSuccess;
    //pulled waypoints
    std::vector< mavros::Waypoint > waypoints;

    //ros related
    ros::NodeHandle nh;
    //subscribers
    ros::Subscriber sub_posi;
    ros::Subscriber sub_posi_local;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_local;
    ros::Subscriber sub_att;
    ros::Subscriber sub_wps;
    //service
    ros::ServiceClient client_wp_pull;
    ros::ServiceClient client_wp_push;

    //callback functions
    void posiCb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void posiLocalCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void velCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void localCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void attCb(const sensor_msgs::Imu::ConstPtr& msg);
    void wpsCb(const mavros::WaypointList::ConstPtr &msg);

    void PullandSendWps();
    bool WaypointsPull();
};

}
