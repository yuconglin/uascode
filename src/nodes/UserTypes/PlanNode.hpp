#pragma once
//user_types
#include "PathGenerator.hpp"
//user structs
#include "Planner/UserStructs/PlaneStateSim.h"
#include "Planner/UserStructs/obstacle3D.h"
#include "Planner/UserStructs/MissionSimPt.h"
#include "UserStructs/GlobalPosi.h"
#include "UserStructs/PlaneAtt.h"
#include "UserStructs/GoalSetPt.h"
//ros msg header
#include "uascode/PosSetPoint.h"
#include "uascode/GlobalPos.h" //next to create the ros msgs
#include "uascode/PlaneAttitude.h"
#include "uascode/MultiObsMsg.h"
//std lib
#include <vector>

namespace UasCode{

class PlanNode{
 public:
   //constructor not defined yet
   PlanNode();

   PathGenerator path_gen;
   //inline void SetTimeLimit(const double _t_limit){t_limit= _t_limit;}
   void SetTimeLimit(const double _t_limit);
   inline void SetWpR(const double _r){this->wp_r= _r;}
   //working part
   int working();

 private:
   enum possible_cases{NORMAL,PATH_READY,PATH_GEN,PATH_CHECK,PATH_RECHECK,WAIT_STATE,ARRIVED};
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
   //wp ros msg to send
   uascode::PosSetPoint set_pt;
   //obstacles
   std::vector<UserStructs::obstacle3D> obss; 
   //waypoint vector storage
   std::vector<UserStructs::GoalSetPt> waypoints;
   //time limit for planning
   double t_limit;
   //radius for wp
   double wp_r;
   //ros related
   ros::NodeHandle nh;
   //publisher
   ros::Publisher pub_interwp;
   //subscribers
   ros::Subscriber sub_obss;
   ros::Subscriber sub_pos;
   ros::Subscriber sub_att;
   ros::Subscriber sub_goal;
   //callback functions
   void obssCb(const uascode::MultiObsMsg::ConstPtr& msg);
   void posCb(const uascode::GlobalPos::ConstPtr& msg);
   void attCb(const uascode::PlaneAttitude::ConstPtr& msg);
   void goalCb(const uascode::PosSetPoint::ConstPtr& msg);
   //other functions
   void GetCurrentSt();
   void GetGoalWp();
   bool CheckGoalChange();
};

};//namespace ends
