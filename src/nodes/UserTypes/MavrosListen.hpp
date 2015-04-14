#pragma once
//user_types
#include "Planner/UserTypes/PathGenerator.hpp"
#include "Planner/UserTypes/ObsHelper.hpp"
//user structs
#include "Planner/UserStructs/PlaneStateSim.h"
//#include "Planner/UserStructs/obstacle3D.h"
#include "nodes/UserStructs/GlobalPosi.h"
#include "nodes/UserStructs/PlaneAtt.h"
#include "nodes/UserStructs/GoalSetPt.h"
#include "nodes/UserStructs/AccelXYZ.h"

//ros msg header
#include "uascode/GlobalPos.h" //next to create the ros msgs
#include "uascode/PlaneAttitude.h"
#include "uascode/MultiObsMsg.h"
#include "uascode/AccelXYZ.h"
#include "uascode/WpCurrent.h"
#include "uascode/WpNumber.h"
#include "uascode/ColliPoint.h"

#include "mavros/Mavlink.h"
#include "mavros/WaypointList.h"
#include "mavros/Waypoint.h"
#include "mavros/State.h"

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
#include "yucong_rosmsg/MultiObsMsg2.h"

namespace UasCode{

class MavrosListen{

public:
    MavrosListen();
    ~MavrosListen();

    void SetTimeLimit(const double _t_limit);
    inline void SetWpR(const double _r){this->wp_r= _r;}
    inline void SetHomeAlt(const double _alt){home_alt= _alt;}

    //load trajectory log file
    void SetLogFileName(const char* filename);
    //load obs distance log
    void SetObsDisFile(const char* filename);

    void working();

private:
    enum possible_cases{NORMAL,PATH_READY,PATH_GEN,PATH_CHECK,PATH_RECHECK,WAIT_STATE,ARRIVED};
    possible_cases situ;

    //path generator
    PathGenerator path_gen;

    //obstacle helper
    std::vector< ObsHelper >* helpers;

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

    //flag to indicate if an inter wp was generated
    bool if_inter_gen;
    bool if_gen_success;
    bool if_inter_exist;
    int seq_current;
    int seq_inter;
    std::string UavState;
    bool if_posi_update;

    //to see if the sent waypoint was received
    bool if_receive;
    //to see if already fail
    bool if_fail;
    //to see if obstacles are updated
    bool if_obss_update;

    //if pulled and sent
    bool IfPullSent;
    bool PullSuccess;
    //pulled waypoints
    std::vector< mavros::Waypoint > waypoints;
    //waypoints flags
    std::vector< bool > flags;
    //intermediate waypoint
    mavros::Waypoint wp_inter;
    //obstacles
    std::vector<UserStructs::obstacle3D> obss;
    //geofence/spacelimit
    UserStructs::SpaceLimit spLimit;
    uascode::ColliPoint colli_pt;

    //time limit for planning
    double t_limit;

    //time step
    double dt;
    //radius for wp
    double wp_r;
    //alt for home waypoint
    double home_alt;
    //threshold ratio for obstacle avoidance
    double thres_ratio;
    //collision prediction return result
    UserStructs::PredictColliReturn colli_return;

    //log for trajectory
    std::ofstream traj_log;
    std::ofstream obdis_log;

    //ros related
    ros::NodeHandle nh;
    //subscribers
    ros::Subscriber sub_obss;
    ros::Subscriber sub_posi;
    ros::Subscriber sub_posi_local;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_local;
    ros::Subscriber sub_att;
    ros::Subscriber sub_wps;
    ros::Subscriber sub_wp_current;
    ros::Subscriber sub_state;
    //service
    ros::ServiceClient client_wp_pull;
    ros::ServiceClient client_wp_push;

    //callback functions
    void obssCb(const yucong_rosmsg::MultiObsMsg2::ConstPtr& msg);
    void posiCb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void posiLocalCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void velCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void localCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void attCb(const sensor_msgs::Imu::ConstPtr& msg);
    void wpsCb(const mavros::WaypointList::ConstPtr &msg);
    void mission_currentCb(const std_msgs::UInt16::ConstPtr &msg);
    void stateCb(const mavros::State::ConstPtr &msg);

    void PullandSendWps();
    bool WaypointsPull();

    //other functions
    void GetCurrentSt();
    void GetObssDis();
    void SetHelpers();
    void PrintSitu();

    int PredictColliNode3(UserStructs::PlaneStateSim &st_current,int seq_current,double t_limit,double thres_ratio,UserStructs::PredictColliReturn& colli_return);
    void MavrosWpToMissionPt(const mavros::Waypoint &wp, UserStructs::MissionSimPt& pt);
    void SendWaypoints();
};

}
