#include "PlanNode.hpp"
#include "common/UserStructs/constants.h"
#include "Planner/UserTypes/Sampler/SamplerPole.hpp"
#include "common/Utils/GetTimeUTC.h"
//std
#include <iostream>
#include <iomanip>
#include <fstream>

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
    //sub_goal= nh.subscribe("position_setpoint",100,&PlanNode::goalCb,this);
    sub_IfRec= nh.subscribe("interwp_receive",100,&PlanNode::ifRecCb,this);
    sub_accel= nh.subscribe("accel_raw_imu",100,&PlanNode::AccelCb,this);
    sub_wp_current= nh.subscribe("waypoint_current",100,&PlanNode::WpCurrCb,this);

    //parameters for controllers
    path_gen.SetTimeLimit(1.0);
    path_gen.SetNinter(5);

    //if in ros?
    path_gen.SetInRos(true);

    //set geofence/spacelimit
    UserStructs::SpaceLimit spacelimit(2000,500);
    //geofence.txt location needs changing.
    spacelimit.LoadGeoFence("/home/yucong/ros_workspace/uascode/bin/geofence.txt");
    this->spLimit= spacelimit;
    path_gen.SetSpaceLimit(spacelimit);

    //set wp_r
    wp_r =30;

    if_receive= false;
    //I use seq_current== -1 to indicate the moment mission starts
    seq_current= -1;

    //parameters for the navigator
    double _Tmax= 12.49*UasCode::CONSTANT_G;
    double _Muav= 29.2; //kg
    double myaw_rate= 20./180*M_PI;
    double mpitch_rate= 10./180*M_PI;
    double _max_speed= 30; //m/s
    double _min_speed= 10; //m/s
    double _max_pitch= 25./180*M_PI;
    double _min_pitch= -20./180*M_PI;

    double dt= 1.;
    double _speed_trim= _max_speed;
    //set
    path_gen.NavUpdaterParams(_Tmax,mpitch_rate,myaw_rate,_Muav,_max_speed,_min_speed,_max_pitch,_min_pitch);

    path_gen.NavTecsReadParams("/home/yucong/ros_workspace/uascode/bin/parameters_sitl.txt");
    path_gen.NavL1SetRollLim(40./180*M_PI);
    path_gen.NavSetDt(dt);
    path_gen.NavSetSpeedTrim(_speed_trim);

    //set sampler parameters
    path_gen.SetSampler(new UserTypes::SamplerPole() );

  }//constructor ends

  void PlanNode::LoadFlightPlan(const char* filename)
  {
    std::ifstream plan_file(filename);
    int line_count= 0;
    //note altitude for each waypoint should be adjusted
    //by adding the height of home waypoint
    if(plan_file.is_open())
    {
       while(plan_file)
       {
         if(line_count>0){
//0	1	0	16	0.00000  	0.000000	0.000000	0.000000	33.422036	-111.926263	30	1
           int all= 12;
           double log[all];
           for(int i=0;i!=all;++i)
               plan_file >> log[i];
               double lat= log[8];
               double lon= log[9];
               double alt= log[10]+home_alt;
               double r= 60;
               double x=0, y=0;
               double h= 200,v=150,alt_rec= 50;
               waypoints.push_back(UserStructs::MissionSimPt(lat,lon,alt,0,r,x,y,h,v,alt_rec) );
               //waypoints.push_back( UserStructs::GoalSetPt(log[8],log[9],log[10]+home_alt) );
         }//if line_count > 0 ends
         ++line_count;
       }//while plan_file ends
    }

    else{
      std::cout<<" flight plan file cannot be loaded"
               << std::endl;
      return;
    }
  }

  void PlanNode::SetTimeLimit(const double _t_limit)
  {
    t_limit= _t_limit;
    path_gen.SetTimeLimit(t_limit);
  }//SetTimeLimit ends

  bool PlanNode::PredictColliNode(UserStructs::PlaneStateSim &st_current,int seq_current,double t_limit)
  {
    NavigatorSim* navigator_pt= path_gen.NavigatorPt();
    bool tt= navigator_pt->PredictColli(st_current,waypoints,wp_init,obss,spLimit,seq_current,t_limit);

    std::cout<<"PredictColliNode: "<< tt << std::endl;
    return tt;
  }

  void PlanNode::working()
  {
    possible_cases situ= NORMAL;
    int seq_current_pre= 0;
    ros::Rate r(10);

    while(ros::ok() )
    { //callback once
      ros::spinOnce();
      //to see if starts
      if(seq_current_pre < 1){

         if(seq_current== 1 )
         {
           wp_init.lat= global_posi.lat;
           wp_init.lon= global_posi.lon;
           wp_init.alt= global_posi.alt;

           std::cout<< "wp_init:" <<" "
                    << std::setprecision(6) << std::fixed
                    << "lat:" << wp_init.lat<< " "
                    << "lon:" << wp_init.lon<< " "
                    << "alt:" << wp_init.alt<< std::endl;

         }
         seq_current_pre= seq_current;
      }

      //check collision in 30 seconds
      GetCurrentSt();

      situ= NORMAL;
      if( PredictColliNode(st_current,seq_current,30) )
          //situ= PATH_GEN;

        //for diffrent cases
        switch(situ){
        case PATH_GEN:
          //get current state
          GetCurrentSt();
          //GetGoalWp();
          //set for path_gen
          //set start state and goal waypoint
          path_gen.SetInitState(st_current.SmallChange(t_limit));
          path_gen.SetGoalWp(waypoints[seq_current]);
          path_gen.SetSampleParas();
          path_gen.SetObs(obss);
          //to generate feasible paths
          path_gen.AddPaths();
          situ= PATH_CHECK;
          break;

        case PATH_CHECK:
        {
          GetCurrentSt();
          UserStructs::MissionSimPt inter_wp;
          if(path_gen.PathCheckRepeat(st_current))
          {
           std::cout<<"check ok" << std::endl;
           inter_wp= path_gen.GetInterWp();

           set_pt.lat= inter_wp.lat;
           set_pt.lon= inter_wp.lon;
           set_pt.alt= inter_wp.alt;

           //here we need to add a flag to test if the waypoint is received
           situ= PATH_READY;
           //insert into waypoints
           waypoints.insert(waypoints.begin()+seq_current,inter_wp);
           //situ= PATH_RECHECK;
          }
          else
           situ= NORMAL;
           break;
        }
        case PATH_READY:
          if(!if_receive)
           //this will be sent to pixhawk by MavlinkReceiver node
           pub_interwp.publish(set_pt);
          else
           situ= PATH_RECHECK;
           break;

        case PATH_RECHECK:
           GetCurrentSt();
           if(path_gen.PathCheckSingle(st_current) )
             situ= NORMAL;
           break;

        default:
           break;
        }//switch ends

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
      //std::cout << "obstacle: "<< obs3d.x1 << std::endl;
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
      /*
      std::cout<< "global_posi:"
               << global_posi.lat<< " "
               << global_posi.lon<< " "
               << global_posi.alt<< " "
               << global_posi.cog<< " "
               << global_posi.speed
               << std::endl;
      */
  }
  
  void PlanNode::attCb(const uascode::PlaneAttitude::ConstPtr& msg)
  {
     plane_att.roll= msg->roll; 
     plane_att.pitch= msg->pitch;
     plane_att.yaw= msg->yaw;
     /*
     std::cout<< "plane_att:"
              << plane_att.roll<< " "
              << plane_att.pitch<< " "
              << plane_att.yaw
              << std::endl;
     */
  }
  
  /*
  void PlanNode::goalCb(const uascode::PosSetPoint::ConstPtr& msg)
  {
     goal_posi.lat= msg->lat;
     goal_posi.lon= msg->lon;
     goal_posi.alt= msg->alt;
  } */

  void PlanNode::ifRecCb(const uascode::IfRecMsg::ConstPtr &msg)
  {
     if_receive= msg->receive;

     std::cout<< "if receive= "<< if_receive << std::endl;
  }

  void PlanNode::AccelCb(const uascode::AccelXYZ::ConstPtr &msg)
  {
     accel_xyz.ax= msg->ax;
     accel_xyz.ay= msg->ay;
     accel_xyz.az= msg->az;
     /*
     std::cout<<"accel from raw imu: "
              << accel_xyz.ax <<" "
              << accel_xyz.ay <<" "
              << accel_xyz.az << std::endl;
     */
  }

  void PlanNode::WpCurrCb(const uascode::WpCurrent::ConstPtr &msg)
  {
     seq_current= msg->wp_current;
     /*
     std::cout<<"current waypoint #: "
              << seq_current
              << std::endl;
              */
  }

  void PlanNode::GetCurrentSt()
  {
     st_current.t= Utils::GetTimeUTC();
     st_current.lat= global_posi.lat;
     st_current.lon= global_posi.lon;
     //get x,y
     st_current.GetUTM();
     st_current.z= global_posi.alt;
     st_current.speed= global_posi.speed; 
     st_current.yaw= plane_att.yaw;
     st_current.pitch= plane_att.pitch;
     //ax,ay,az will be calcuated in update
     /*
     std::cout<<"st_current:" <<" "
       <<"t:"<<std::setprecision(3)<<st_current.t<<" "
       <<"lat:"<<std::setprecision(3)<<st_current.lat<<" "
       <<"lon:"<<std::setprecision(3)<<st_current.lon<<" "
       <<"alt:"<<std::setprecision(3)<<st_current.z<<" "
       <<"spd:"<<std::setprecision(3)<<st_current.speed<<" "
       <<"yaw:"<<std::setprecision(3)<<st_current.yaw*180/M_PI<<" "
       <<"pitch:"<<std::setprecision(3)<<st_current.pitch*180/M_PI
       << std::endl;
     */
  }

  void PlanNode::GetGoalWp()
  {
    goal_wp= UserStructs::MissionSimPt(goal_posi.lat,goal_posi.lon,goal_posi.alt,0.,wp_r,0,0,200,250,20); 
    goal_wp.GetUTM();
  }
  /*
  bool PlanNode::CheckGoalChange()
  {
    if(fabs(goal_wp.lat-goal_pre.lat)> 0.0001 ||
       fabs(goal_wp.lon-goal_pre.lon)> 0.0001 ||
       fabs(goal_wp.alt-goal_pre.alt)> 0.5)
     return true;
    return false;

  } */

}//namespace ends
