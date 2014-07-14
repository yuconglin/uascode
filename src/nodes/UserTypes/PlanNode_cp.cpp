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

  }//constructor ends

  void PlanNode::working()
  {
    possible_cases situ= NORMAL;

    ros::Rate r(10);
    while(ros::ok() )
    { //callback once
      ros::spinOnce();
      //print to examine subscribers

      //only need to react when obstacles present
      if(!obss.empty())
      {
	//get current state
	GetCurrentSt();
	//for diffrent cases
	switch(situ){
	case NORMAL:
	//calculate the time until collision
          double t=TimeColli(st_current,obss);
          if(t < 30+2*t_limit){
            //do the planning
	    //if no path, jump to the next waypoint
	    //if still no path or the last wp, local planner

	  }//t<30+2*t_limit ends
	  break;

	}//switch ends
      }//if obss not empty() ends
      
      
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

};//namespace ends
