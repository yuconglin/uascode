#pragma once
//user_types
#include "Planner/UserTypes/PathGenerator.hpp"
//user structs
#include "Planner/UserStructs/PlaneStateSim.h"
#include "Planner/UserStructs/obstacle3D.h"
#include "Planner/UserStructs/MissionSimPt.h"
#include "nodes/UserStructs/GlobalPosi.h"
#include "nodes/UserStructs/PlaneAtt.h"
#include "nodes/UserStructs/GoalSetPt.h"
#include "nodes/UserStructs/AccelXYZ.h"
//ros msg header
#include "uascode/PosSetPoint.h"
#include "uascode/GlobalPos.h" //next to create the ros msgs
#include "uascode/PlaneAttitude.h"
#include "uascode/MultiObsMsg.h"
#include "uascode/IfRecMsg.h"
#include "uascode/AccelXYZ.h"
#include "uascode/WpCurrent.h"
#include "uascode/IfMavlinkGood.h"
#include "uascode/IfCollision.h"

#include "ros/ros.h"
//std lib
#include <vector>

namespace UasCode{

class PlanNode{
 public:
   //constructor not defined yet
   PlanNode();

   //inline void SetTimeLimit(const double _t_limit){t_limit= _t_limit;}
   void SetTimeLimit(const double _t_limit);
   inline void SetWpR(const double _r){this->wp_r= _r;}
   inline void SetHomeAlt(const double _alt){home_alt= _alt; }

   void SetLogFileName(const char* filename);
   //load flight plan from txt
   void LoadFlightPlan(const char* filename);
   //working part
   void working();
   //to test predicting
   void predicting();

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
   //goal set point
   UserStructs::GoalSetPt goal_posi;
   //ACCEL
   UserStructs::AccelXYZ accel_xyz;
   //if mavlink communication exsits
   bool if_mavlink;
   //Current Waypoint
   int seq_current;

   //flag to indicate if an inter wp was generated
   bool if_inter_gen;
   int seq_inter;

   //to see if the sent waypoint was received
   bool if_receive;

   //wp ros msg to send
   uascode::PosSetPoint set_pt;
   uascode::IfCollision IfColliMsg;

   //obstacles
   std::vector<UserStructs::obstacle3D> obss; 
   //geofence/spacelimit
   UserStructs::SpaceLimit spLimit;

   //waypoint vector storage
   std::vector<UserStructs::MissionSimPt> waypoints;
   //the first actual waypoint after take-off
   UserStructs::GoalSetPt wp_init;
   //time limit for planning
   double t_limit;

   //radius for wp
   double wp_r;
   //alt for home waypoint
   double home_alt;

   //log for trajectory
   std::ofstream traj_log;

   //ros related
   ros::NodeHandle nh;
   //publisher
   ros::Publisher pub_interwp;
   ros::Publisher pub_if_colli;
   //subscribers
   ros::Subscriber sub_obss;
   ros::Subscriber sub_pos;
   ros::Subscriber sub_att;
   ros::Subscriber sub_IfRec;
   ros::Subscriber sub_accel;
   ros::Subscriber sub_wp_current;
   //ros::Subscriber sub_if_mavlink;
   //ros::Subscriber sub_goal;

   //callback functions
   void obssCb(const uascode::MultiObsMsg::ConstPtr& msg);
   void posCb(const uascode::GlobalPos::ConstPtr& msg);
   void attCb(const uascode::PlaneAttitude::ConstPtr& msg);
   //void goalCb(const uascode::PosSetPoint::ConstPtr& msg);
   void ifRecCb(const uascode::IfRecMsg::ConstPtr& msg);
   void AccelCb(const uascode::AccelXYZ::ConstPtr& msg);
   void WpCurrCb(const uascode::WpCurrent::ConstPtr& msg);
   //void IfMavlinkCb(const uascode::IfMavlinkGood::ConstPtr& msg);

   //other functions
   void GetCurrentSt();
   void GetObssDis();
   void GetGoalWp();
   bool CheckGoalChange();
   bool PredictColliNode(UserStructs::PlaneStateSim &st_current,int seq_current,double t_limit);
};

}//namespace ends
