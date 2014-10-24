#pragma once
//user_types
#include "Planner/UserTypes/PathGenerator.hpp"
//user structs
#include "Planner/UserStructs/PlaneStateSim.h"
#include "Planner/UserStructs/obstacle3D.h"
#include "Planner/UserStructs/MissionSimFlagPt.h"
#include "nodes/UserStructs/GlobalPosi.h"
#include "nodes/UserStructs/PlaneAtt.h"
//ros msg headers
#include "uascode/MultiObsMsg.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"G
#include "ros/ros.h"

#include <vector>

namespace UasCode{

class PlanNode3{
public:
    PlanNode3();
    ~PlanNode3();

private:
    enum possible_cases{NORMAL,PATH_READY,PATH_GEN,PATH_CHECK,PATH_RECHECK,WAIT_STATE,ARRIVED};
    //path generator
    PathGenerator path_gen;
    //plane state current
    UserStructs::PlaneStateSim st_current;
    //plane goal
    UserStructs::MissionSimPt goal_wp;
    //plane position
    UserStructs::GlobalPosi global_posi;
    //plane attitude
    UserStructs::PlaneAtt plane_att;
    //current waypoint
    int seq_current;

    //flag to indicate if an inter wp was generated
    bool if_inter_gen;
    bool if_inter_exist;
    int seq_inter;

    //to see if the sent waypoint was received
    bool if_receive;

    //custom mavlink message to send to the autopilot
    //adsb obstacles
    //collision point

    //obstacles
    std::vector<UserStructs::obstacle3D> obss;
    //geofence/spacelimit
    UserStructs::SpaceLimit spLimit;

    //waypoint vector storage
    std::vector<UserStructs::MissionSimFlagPt> FlagWayPoints;
    //the first actual waypoint after take-off
    UserStructs::GoalSetPt wp_init;
    //time limit for planning
    double t_limit;

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
    //publisher
    ros::Publisher pub_mavlink;
    //subscribers
    ros::Subscriber sub_obss;
    ros::Subscriber sub_posi;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_att;
    //service
    ros::ServiceClient client_wp;

    //callback functions
    void obssCb(const uascode::MultiObsMsg::ConstPtr& msg);
    void posiCb(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void velCb(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void attCb(const sensor_msgs::Imu::ConstPtr& msg);
    //other functions
    void GetCurrentSt();
    void GetObssDis();
    int PredictColliNode2(UserStructs::PlaneStateSim &st_current,int seq_current,double t_limit,double thres_ratio,UserStructs::PredictColliReturn& colli_return);
};

}//namespace UasCode ends
