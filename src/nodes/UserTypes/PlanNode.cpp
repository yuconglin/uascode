#include "PlanNode.hpp"
//std
#include <iostream>
#include <iomanip>

namespace UasCode{
//constructor
  PlanNode::PlanNode()
  {
    //publisher
    pub_interwp= nh.advertise<uascode::PosSetPoint>("inter_wp",100);
    //subscriber
    sub_obss= nh.subscribe("multi_obstacles",100,&PlanNode::obssCb,this);
    sub_pos= nh.subscribe("global_position",100,&PlanNode::posCb,this);
    sub_att= nh.subscribe("plane_att",100,&PlanNode::attCb,this);
    sub_goal= nh.subscribe("position_setpoint",100,&PlanNode::goalCb,this);
    //parameters for controllers
    path_gen.SetTimeLimit(1.0);
    path_gen.SetNinter(5);
    //if in ros?
    path_gen.SetInRos(true);
    //set wp_r
    wp_r =30;
//parameters for the navigator
  //parameters
  double _Tmax= 6*UasCode::CONSTANT_G;
  double _Muav= 3; //kg
  double myaw_rate= 10./180*M_PI;
  double mpitch_rate= 5./180*M_PI;
  double _max_speed= 14; //m/s
  double _min_speed= 7; //m/s
  double _max_pitch= 20./180*M_PI;
  double _min_pitch= -20./180*M_PI;
  
  double dt= 1.;
  double _speed_trim= 9.;
  //set
  path_gen.NavUpdaterParams(_Tmax,mpitch_rate,myaw_rate,_Muav,_max_speed,_min_speed,_max_pitch,_min_pitch);

  path_gen.NavTecsReadParams("parameters.txt");
  path_gen.NavL1SetRollLim(10./180*M_PI);
  path_gen.NavSetDt(dt);
  path_gen.NavSetSpeedTrim(_speed_trim);
  //set sampler parameters
  path_gen.SetSampler(new UserTypes::SamplerPole() );


  }//constructor ends

  void PlanNode::SetTimeLimit(const double _t_limit)
  {
    t_limit= _t_limit;
    path_gen.SetTimeLimit(t_limit);
  }//SetTimeLimit ends

  void PlanNode::working()
  {
    possible_cases situ= NORMAL;
    UserStructs::GoalSetPt goal_pre;

    ros::Rate r(10);
    while(ros::ok() )
    { //callback once
      ros::spinOnce();
      //print to examine subscribers

      //only need to react when obstacles present
      if(!obss.empty())
      {
	GetGoalWp();
        if(CheckGoalChange() )
	  situ= NORMAL;
	//for diffrent cases
	switch(situ){
	 case NORMAL:
          //get current state
	  GetCurrentSt();
          //GetGoalWp();	
          //set for path_gen
	  //set start state and goal waypoint
          path_gen.SetInitState(st_current.SmallChange(t_limit));
          path_gen.SetGoalWp(goal_wp);
	  path_gen.SetSampleParas();
	  path_gen.SetObs(obss);
          //to generate feasible paths
          path_gen.AddPaths();
	  situ= PATH_CHECK;
	  break;

	 case PATH_CHECK:
	  GetCurrentSt();
          UserStructs::MissionSimPt inter_wp;
          if(path_gen.PathCheckRepeat(st_current))
	  {
           std::cout<<"check ok" << std::endl;
           inter_wp= path_gen.GetInterWp();
	   //uascode::PosSetPoint set_pt;
	   set_pt.lat= inter_wp.lat;
	   set_pt.lon= inter_wp.lon;
	   set_pt.alt= inter_wp.alt;
	   //this will be sent to pixhawk by MavlinkReceiver node
	   pub_interwp.publish(set_pt);
	   //situ= PATH_READY;
	   situ= PATH_RECHECK;
	  }
	  else
	   situ= NORMAL;
          break;

         case PATH_READY:
          
	  break;

         case PATH_RECHECK:
          GetCurrentSt();
          if(path_gen.PathCheckSingle(st_current) )
	    situ= NORMAL;
	  break;
         
	}//switch ends
      }//if obss not empty() ends
      
      goal_pre= goal_posi; 
      r.sleep();
    }//while ends

  }//working

  //callback functions
  void PlanNode::obssCb(const uascode::MultiObsMsg::ConstPtr& msg)
  {
    if(!obss.empty() ) obss.clear();
    for(int i=0;i!= msg->MultiObs.size();++i)
    {
      UserStructs::obstacle3D obs3d(
            msg->MultiObs[i].x1,
	    msg->MultiObs[i].x2,
	    msg->MultiObs[i].head_xy,
	    msg->MultiObs[i].speed,
	    msg->MultiObs[i].x3,
	    msg->MultiObs[i].v_vert,
	    msg->MultiObs[i].t,
	    msg->MultiObs[i].r,0,
	    msg->MultiObs[i].hr,0);
      obss.push_back(obs3d);
    }//for ends

  }//obssCb ends

  void PlanNode::posCb(const uascode::GlobalPos::ConstPtr& msg)
  {
      global_posi.lat= msg->lat;
      global_posi.lon= msg->lon;
      global_posi.alt= msg->alt;
      global_posi.cog= msg->cog;
      global_posi.speed= msg->speed;
  }
  
  void PlanNode::attCb(const uascode::PlaneAttitude& msg)
  {
     plane_att.roll= msg->roll; 
     plane_att.pitch= msg->pitch;
     plane_att.yaw= msg->yaw;
  }
  
  void PlanNode::goalCb(const uascode::PosSetPoint& msg)
  {
     goal_posi.lat= msg->lat;
     goal_posi.lon= msg->lon;
     goal_posi.alt= msg->alt;
  }

  void PlanNode::GetCurrentSt()
  {
     st_current.t=GetTimeUTC();
     st_current.lat= global_posi.lat;
     st_current.lon= global_posi.lon;
     //get x,y
     st_current.GetUTM();
     st_current.z= global_posi.alt;
     st_current.speed= global_posi.speed; 
     st_current.yaw= plane_att.yaw;
     st_current.pitch= plane_att.pitch;
     //ax,ay,az will be calcuated in update
     std::cout<<"st_current:" <<" "
       <<"t:"<<std::setprecision(3)<<st_current.t<<" "
       <<"lat:"<<std::setprecision(3)<<st_current.lat<<" "
       <<"lon:"<<std::setprecision(3)<<st_current.lon<<" "
       <<"alt:"<<std::setprecision(3)<<st_current.z<<" "
       <<"spd:"<<std::setprecision(3)<<st_current.speed<<" "
       <<"yaw:"<<std::setprecision(3)<<st_current.yaw*180/M_PI<<" "
       <<"pitch:"<<std::setprecision(3)<<st_current.pitch*180/M_PI
       << std::endl;

  }

  void PlanNode::GetGoalWP()
  {
    goal_wp= UserStructs::MissionSimPt(goal_posi.lat,goal_posi.lon,goal_posi.alt,0.,wp_r,0,0,200,250,20); 
    goal_wp.GetUTM();
  }

  bool PlanNode::CheckGoalChange()
  {
    if(fabs(goal_wp.lat-goal_pre.lat)> 0.0001 ||
       fabs(goal_wp.lon-goal_pre.lon)> 0.0001 ||
       fabs(goal_wp.alt-goal_pre.alt)> 0.5)
     return true;
    return false;

  }

};//namespace ends
