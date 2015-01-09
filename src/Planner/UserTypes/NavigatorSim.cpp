#include "NavigatorSim.hpp"
#include "Planner/Utils/DubinsLength3D.h"
#include "Planner/Utils/DubinsLength2.h"
#include "Planner/Utils/CheckCollision.h"
#include "common/Utils/UTMtransform.h"
#include "common/Utils/YcLogger.h"
#include "common/Utils/GeoUtils.h"
//other
#include <iomanip>

namespace{
Utils::LoggerPtr s_logger(Utils::getLogger("uascode.NavigatorSim.YcLogger"));
}

namespace UasCode{

NavigatorSim::NavigatorSim(const char* act_name,const char* state_name){
  fs_act.clear();
  fs_act.open(act_name,std::ofstream::out | std::ofstream::in | std::ofstream::trunc);
  fs_state.clear();
  fs_state.open(state_name,std::ofstream::out | std::ofstream::in | std::ofstream::trunc);
  speed_trim= 30;
  check_step= 457.2;//in meters = 1500 ft, the medius score separation
  N_inter= 5;
}

void NavigatorSim::UpdaterSetParams(double _Tmax,double _prate,double _yrate,double _M,double _max_spd,double _min_spd,double _max_pitch,double _min_pitch){
  updater.SetParams( _Tmax, _prate, _yrate, _M,_max_spd,_min_spd,_max_pitch,_min_pitch);
  tecs.set_mpitch_rate( _prate);
  tecs.set_pitch_min(_min_pitch);
  tecs.set_pitch_max(_max_pitch);
  //turn radius
  turn_radius= _max_spd/_yrate;
}

void NavigatorSim::TECsReadParams(const char* filename){
  tecs._load_param_from_file(filename);
}

void NavigatorSim::L1SetRollLim(float _roll_lim){
  l1_control.set_l1_roll_limit(_roll_lim);  
}

void NavigatorSim::SetDt(double _dt){
  dt= _dt;
  tecs.SetTinc(_dt);
}

void NavigatorSim::EnableAirspd(){
  tecs.enable_airspeed(true);
}

void NavigatorSim::ClearRec(){ states_rec.clear(); }

void NavigatorSim::PropagateStep(UserStructs::PlaneStateSim& st_start,UserStructs::PlaneStateSim& st_end,arma::vec::fixed<2> pt_A,UserStructs::MissionSimPt& pt_target)
{
  st_start.CheckConvert();

  double spd= st_start.speed;
  double pitch= st_start.pitch;
  double yaw= st_start.yaw;
  //ground speed vector
  arma::vec::fixed<2> gnd_speed;
  //current position
  arma::vec::fixed<2> curr_posi;
  //target point
  arma::vec::fixed<2> pt_B;
  //assign
  gnd_speed<< spd*cos(pitch)*sin(M_PI/2-yaw)<< spd*cos(pitch)*cos(M_PI/2-yaw);
  curr_posi<< st_start.lat<< st_start.lon;
  pt_B<< pt_target.lat<< pt_target.lon;
  //l1 controller, navigate_waypoints

  l1_control.navigate_waypoints(pt_A,pt_B,curr_posi,gnd_speed);
  double nav_roll= l1_control.nav_roll();
  double nav_yaw= l1_control.nav_bearing(); 

  //tecs update_50hz 
  arma::vec::fixed<3> accel;
  accel<< st_start.ax<< st_start.ay<< st_start.az;

  tecs.update_50hz(st_start.z,accel);

  //update_pitch_throttle
  tecs.update_pitch_throttle(st_start.pitch,st_start.yaw,st_start.z,pt_target.alt,speed_trim,st_start.speed,1.,false);
  double dem_pitch= tecs.get_pitch_demand();
  double dem_thr= tecs.get_throttle_demand();

  //log actions
  /*
  fs_act<< nav_roll*180./M_PI<<" "
        << nav_yaw*180./M_PI<<" "
	<< dem_pitch*180./M_PI<<" "
	<< dem_thr << std::endl;
  */
  //update based on action

  st_end= updater.update(st_start,nav_roll,nav_yaw,dem_pitch,dem_thr,dt);
  //log states
  /*
  fs_state<< std::setprecision(5)<< std::fixed<< st_end.t<<" "
          << st_end.x<<" "
	  << std::setprecision(7)<< st_end.y<<" "
	  << st_end.lat<<" "
	  << st_end.lon<<" "
	  << st_end.z<<" "
	  << st_end.speed<<" "
	  << st_end.yaw/M_PI*180.<<" "
	  << st_end.pitch/M_PI*180.<<" "
	  << st_end.ax<<" "<< st_end.ay<<" "<< st_end.az<<" "
	  << sqrt(pow(st_end.x-pt_target.x,2)
	         +pow(st_end.y-pt_target.y,2))<<" "
	  << std::endl;
  */
}

int NavigatorSim::PropagateWp(UserStructs::PlaneStateSim& st_start,
		 UserStructs::PlaneStateSim& st_end,
		 arma::vec::fixed<2> pt_A,
		 UserStructs::MissionSimPt& pt_target)
{
   double rho= turn_radius;
   
   st_start.CheckConvert();
   pt_target.CheckConvert();

   double yaw1= atan2(pt_target.y-st_start.y, pt_target.x-st_start.x);
   double Ld= Utils::DubinsLength3D(st_start.x,st_start.y,st_start.z,
                              M_PI/2-st_start.yaw,
                              pt_target.x,pt_target.y,pt_target.alt,yaw1,
                              rho);

   //update util reached
   UserStructs::PlaneStateSim st_now = st_start;
   UserStructs::PlaneStateSim st_next;
   double length=0.;//length travelled
   int result;//0: arrived by closeness, 1: arrived by length

   //the first state needs to be logged
   fs_state<< std::setprecision(5)<< std::fixed<< st_now.t<<" "
           << st_now.x<<" "
       << std::setprecision(7)<< st_now.y<<" "
       << st_now.lat<<" "
       << st_now.lon<<" "
       << st_now.z<<" "
       << st_now.speed<<" "
       << st_now.yaw/M_PI*180.<<" "
       << st_now.pitch/M_PI*180.<<" "
       << st_now.ax<<" "<< st_now.ay<<" "<< st_now.az<<" "
       << sqrt(pow(st_now.x-pt_target.x,2)
              +pow(st_now.y-pt_target.y,2))<<" "
       << std::endl;

   //loop
   while(1){
     PropagateStep(st_now,st_next,pt_A,pt_target);

     fs_state<< std::setprecision(5)<< std::fixed<< st_next.t<<" "
             << st_next.x<<" "
             << std::setprecision(7)<< st_next.y<<" "
             << st_next.lat<<" "
             << st_next.lon<<" "
             << st_next.z<<" "
             << st_next.speed<<" "
             << st_next.yaw/M_PI*180.<<" "
             << st_next.pitch/M_PI*180.<<" "
             << st_next.ax<<" "<< st_next.ay<<" "<< st_next.az<<" "
             << sqrt(pow(st_next.x-pt_target.x,2)
                     +pow(st_next.y-pt_target.y,2))<<" "
             << std::endl;

     if(pt_target.SeeArrive(st_next.x,st_next.y,st_next.z) ){
       result= 0;
       break;
     }//if ends

     length+= sqrt( pow(st_now.x-st_next.x,2)
                    + pow(st_now.y-st_next.y,2)
                    + pow(st_now.z-st_next.z,2)
                  );

     if(length >1.1*Ld){
       result= 1;
       break;
     }

     st_now= st_next;

   }//while ends
   st_end= st_next;
   return result;
}

int NavigatorSim::PropWpCheck(UserStructs::PlaneStateSim& st_start,
                UserStructs::PlaneStateSim& st_end,
                arma::vec::fixed<2> pt_A,
                UserStructs::MissionSimPt& pt_target,
                std::vector<UserStructs::obstacle3D> obstacles,
                double &length)
{
   double rho= turn_radius;
   
   st_start.CheckConvert();
   pt_target.CheckConvert();

   double yaw1= atan2(pt_target.y-st_start.y, pt_target.x-st_start.x);
   double Ld= Utils::DubinsLength3D(st_start.x,st_start.y,st_start.z,
			      M_PI/2-st_start.yaw,
			      pt_target.x,pt_target.y,pt_target.alt,yaw1,
			      rho);
   //update util reached
   UserStructs::PlaneStateSim st_now = st_start;
   UserStructs::PlaneStateSim st_next;
   length=0.;//length travelled
   int result;//-1:collision, 0: arrived by closeness, 1: arrived by length

   //loop
    while(1){
     PropagateStep(st_now,st_next,pt_A,pt_target);

     if(pt_target.SeeArrive(st_next.x,st_next.y,st_next.z) ){
       result= 0;
       break;
     }//if ends

     for(int i=0;i!= obstacles.size();++i){

        if(Utils::CheckCollision(st_next,obstacles[i])==1){
          result = -1;
	  break;
	}

     }
     if(result== -1) break;

     length+= sqrt( pow(st_now.x-st_next.x,2)
		  + pow(st_now.y-st_next.y,2)
		  + pow(st_now.z-st_next.z,2)
	 );
     fs_state <<"length/Ld: "<< length/Ld<< std::endl;

     if(length >1.1*Ld){
       result= 1;
       break;
     }

     st_now= st_next;

   }//while ends
   st_end= st_next;
   return result;
 
}//PropWpCheck ends

 int NavigatorSim::PropWpCheck2(UserStructs::PlaneStateSim& st_start,
                  UserStructs::PlaneStateSim& st_end,
                  arma::vec::fixed<2> pt_A,
                  UserStructs::MissionSimPt& pt_target,
                  std::vector<UserStructs::obstacle3D> obstacles,
                                UserStructs::SpaceLimit spacelimit,
                                double &length,
                                int option)
{   //clear previous record
    //first check closeness
    if(pt_target.SeeArriveSphere(st_start.x,st_start.y,st_start.z) )
    {
      st_end= st_start;
      return 0;
    }

    states_rec.clear();
    states_part_rec.clear();
    
    states_rec.push_back(st_start);
    states_part_rec.push_back(UserStructs::StateNode(st_start,0));

    double rho= turn_radius;
    st_start.CheckConvert();
    pt_target.CheckConvert();
    
    double yaw1= atan2(pt_target.y-st_start.y, pt_target.x-st_start.x);
    double Ld= Utils::DubinsLength2(st_start.x,st_start.y,st_start.z,
			       M_PI/2-st_start.yaw,
			       pt_target.x,pt_target.y,pt_target.alt,yaw1,
			       rho,GetMaxPitch() );

    //update util reached
    UserStructs::PlaneStateSim st_now = st_start;

    UserStructs::PlaneStateSim st_next;
    length=0.;//length travelled

    int result=-2;//-1:collision, 0: arrived by closeness, 1: arrived by length, 2: location past by

    int Nec= 0;

    //loop
    while(1){  
      PropagateStep(st_now,st_next,pt_A,pt_target);

      if(Utils::location_passed_point(st_next,pt_A,pt_target)){
          result =2;
          break;
      }

      bool if_arrive= false;
      if(option==0) 
        if_arrive= pt_target.SeeArriveSphere(st_next.x,st_next.y,st_next.z); 
      else
        if_arrive= pt_target.SeeArrive(st_next.x,st_next.y,st_next.z);

      if(if_arrive){
          result= 0;
          break;
      }//if ends

      length+= sqrt( pow(st_now.x-st_next.x,2)
            + pow(st_now.y-st_next.y,2)
            + pow(st_now.z-st_next.z,2)
            );

      //if( (int)(length/check_step) > Nec )
      {
          //check for obstacles
          for(int i=0;i!= obstacles.size();++i){

              //if(Utils::CheckCollisionSet(st_next,obstacles[i])==1)
if(Utils::CheckCollision(st_next,obstacles[i])==1)
              {
                  result = -1;
                  break;
              }

          }

          //check for spacelimit
          if(result!= -1 && !spacelimit.TellIn(st_next.x,st_next.y,st_next.z) ){
              result = -1;
          }

          if(result!=-1 && Nec % N_inter==0) {
              states_part_rec.push_back(UserStructs::StateNode(st_next,length));
          }
          if(result== -1) break;
      }

      Nec= (int)(length/check_step); 
            
      states_rec.push_back(st_next);

      if(length >1.1*Ld){
          result= 1;
          break;
      }
      st_now = st_next;
    }//while ends
    st_end= st_next;
    return result;

}//PropWpCheck2 ends

int NavigatorSim::PropWpCheckTime(UserStructs::PlaneStateSim& st_start,
                     UserStructs::PlaneStateSim& st_end,
                     arma::vec::fixed<2> pt_A,
                     UserStructs::MissionSimPt& pt_target,
                     std::vector<UserStructs::obstacle3D> obstacles,
                     UserStructs::SpaceLimit spacelimit,
                     double &length,double t_horizon,double& t_left,
                     int option,double thres_ratio,int& obs_idx)
 {
    if(pt_target.SeeArriveSphere(st_start.x,st_start.y,st_start.z) )
    {
      st_end= st_start;
      UASLOG(s_logger,LL_DEBUG,"arrive end");
      return 0;
    }

    states_rec.clear();
    states_part_rec.clear();

    states_rec.push_back(st_start);
    states_part_rec.push_back(UserStructs::StateNode(st_start,0));

    double rho= turn_radius;
    st_start.CheckConvert();
    pt_target.CheckConvert();

    double yaw1= atan2(pt_target.y-st_start.y, pt_target.x-st_start.x);
    double Ld= Utils::DubinsLength2(st_start.x,st_start.y,st_start.z,
                   M_PI/2-st_start.yaw,
                   pt_target.x,pt_target.y,pt_target.alt,yaw1,
                   rho,GetMaxPitch() );

    //update util reached
    UserStructs::PlaneStateSim st_now = st_start;

    UserStructs::PlaneStateSim st_next;
    length=0.;//length travelled

    int result=-2;//-1:collision, 0: arrived by closeness, 1: arrived by length, 2: time reached, 3: location passed by

    int Nec= 0;

    //loop
    while(1){

      PropagateStep(st_now,st_next,pt_A,pt_target);
      bool if_arrive= false;
      if(option==0)
        if_arrive= pt_target.SeeArriveSphere(st_next.x,st_next.y,st_next.z);
      else
        if_arrive= pt_target.SeeArrive(st_next.x,st_next.y,st_next.z);

      if(if_arrive){
          UASLOG(s_logger,LL_DEBUG,"arrived");
          result= 0;
          break;
      }//if ends

      if(Utils::location_passed_point(st_next,pt_A,pt_target)){
          result =3;
          break;
      }

      length+= sqrt( pow(st_now.x-st_next.x,2)
            + pow(st_now.y-st_next.y,2)
            + pow(st_now.z-st_next.z,2)
            );

      //if( (int)(length/check_step) > Nec )
      {

          for(int i=0;i!= obstacles.size();++i){

              if(Utils::CheckCollision2(st_next,obstacles[i],thres_ratio)==1){
                  result = -1;
                  obs_idx= i;
                  break;
              }

          }

          //check for spacelimit
          if(result!= -1 && !spacelimit.TellIn(st_next.x,st_next.y,st_next.z) ){
              result = -1;
          }

          if(result!=-1 && Nec % N_inter==0) {
              states_part_rec.push_back(UserStructs::StateNode(st_next,length));
          }
          if(result== -1){
              UASLOG(s_logger,LL_DEBUG,"result -1, break");
              break;
          }
      }

      Nec= (int)(length/check_step);

      //check for t_horizon
      if(st_next.t- st_start.t >= t_horizon){
          UASLOG(s_logger,LL_DEBUG,"result 2, break");
          result= 2;
          break;
      }

      states_rec.push_back(st_next);

      if(length >1.1*Ld){
          result= 1;
          break;
      }
      st_now = st_next;
    }//while ends

    st_end= st_next;

    t_left= t_horizon-(st_end.t-st_start.t);

    return result;
 }

bool NavigatorSim::PredictColli(UserStructs::PlaneStateSim &st_current,
                                std::vector<UserStructs::MissionSimPt> waypoints,
                                UserStructs::GoalSetPt init_pt,
                                std::vector<UserStructs::obstacle3D> obstacles,
                                UserStructs::SpaceLimit spacelimit,
                                int seq_current, double t_limit,
                                double thres_ratio)
{
    //true:collision, false:no collision
    UserStructs::PlaneStateSim st_start= st_current, st_next;
    arma::vec::fixed<2> pt_start;
    UserStructs::MissionSimPt pt_target;
    double length=0;
    double t_left;
    int result;
    int obs_idx=-1;

    for(int i= seq_current;i!= waypoints.size();++i)
    {
        if(i == 1)
            pt_start << init_pt.lat << init_pt.lon;
        else
            pt_start << waypoints[i-1].lat << waypoints[i-1].lon;

        UASLOG(s_logger,LL_DEBUG,"test wp: "<< i);
        pt_target= waypoints[i];

        result= PropWpCheckTime(st_start,st_next,pt_start,pt_target,
                                obstacles,spacelimit,
                                length,t_limit,t_left,1,thres_ratio,obs_idx);

        if(result == -1){
            UASLOG(s_logger,LL_DEBUG,"predict colli time: "<< st_next.t-st_start.t);
            return true;
        }

        if(result == 2) {
            return false;
        }

        t_limit= t_left;
        st_start= st_next;
    }//for int i ends

    return false;
}

bool NavigatorSim::PredictColli2(UserStructs::PlaneStateSim &st_current,
                                 std::vector<UserStructs::MissionSimFlagPt> waypoints,
                                 UserStructs::GoalSetPt init_pt,
                                 std::vector<UserStructs::obstacle3D> obstacles,
                                 UserStructs::SpaceLimit spacelimit,
                                 int seq_current, double t_limit,
                                 double thres_ratio, UserStructs::PredictColliReturn& colli_return)
{
    //true:collision, false:no collision
    UserStructs::PlaneStateSim st_start= st_current, st_next;
    arma::vec::fixed<2> pt_start;
    UserStructs::MissionSimPt pt_target;
    double length=0;
    double t_left;
    int result;
    int obs_idx = -1;

    for(int i= seq_current;i!= waypoints.size();++i)
    {
        if(i == 1)
            pt_start << init_pt.lat << init_pt.lon;
        else
            pt_start << waypoints[i-1].pt.lat << waypoints[i-1].pt.lon;

        UASLOG(s_logger,LL_DEBUG,"test wp: "<< i);
        pt_target= waypoints[i].pt;

        result= PropWpCheckTime(st_start,st_next,pt_start,pt_target,
                                obstacles,spacelimit,
                                length,t_limit,t_left,1,thres_ratio,obs_idx);

        if(result == -1){
            colli_return.seq_colli= i;
            colli_return.time_colli = st_next.t-st_current.t;
            colli_return.x_colli= st_next.x;
            colli_return.y_colli= st_next.y;
            colli_return.z_colli= st_next.z;
            colli_return.obs_id = obs_idx;
            return true;
        }

        if(result == 2) {
            return false;
        }

        t_limit= t_left;
        st_start= st_next;
    }//for int i ends

    return false;
}

void NavigatorSim::CopyStatesRec(std::vector<UserStructs::PlaneStateSim>& copy_rec)
{
    copy_rec= states_rec;
}

void NavigatorSim::CopyStatePart(std::vector<UserStructs::StateNode>& copy_rec)
{
    copy_rec= states_part_rec;
}

}//namespace ends
