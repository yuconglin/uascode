#include "NavigatorSim.hpp"
#include "Planner/Utils/DubinsLength3D.h"
#include "Planner/Utils/DubinsLength2.h"
#include "Planner/Utils/CheckCollision.h"
#include "common/Utils/UTMtransform.h"
//other
#include <iomanip>

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
  //tecs.set_dt(dt);
  tecs.SetTinc(_dt);
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
  //std::cout<<"pt_A: "<<pt_A(0)<<" "<<pt_A(1)<< std::endl;
  //std::cout<<"pt_B: "<< pt_B(0)<<" "<<pt_B(1)<< std::endl;
  //std::cout<<"curr_posi: "<< curr_posi(0)<<" "<< curr_posi(1)<< std::endl;
  //std::cout<<"gnd_speed: "<< gnd_speed(0)<<" "<< gnd_speed(1)<< std::endl;
  l1_control.navigate_waypoints(pt_A,pt_B,curr_posi,gnd_speed);
  double nav_roll= l1_control.nav_roll();
  double nav_yaw= l1_control.nav_bearing(); 
  //std::cout<<"d_roll: "<< nav_roll/M_PI*180.<<" d_bearing: "<< nav_yaw/M_PI*180.<< std::endl;
  //tecs update_50hz 
  arma::vec::fixed<3> accel;
  accel<< st_start.ax<< st_start.ay<< st_start.az;
  //accel << st_start.ax<< st_start.ay<< 0;
  //std::cout<<"st_start.z: "<< st_start.z<< std::endl;
  tecs.update_50hz(st_start.z,accel);
  //tecs.update_1hz(st_start.z,accel(2) );
  //update_pitch_throttle
  tecs.update_pitch_throttle(st_start.pitch,st_start.z,pt_target.alt,speed_trim,st_start.speed,1.,false);
  double dem_pitch= tecs.get_pitch_demand();
  double dem_thr= tecs.get_throttle_demand();
  //std::cout<<"d_pitch: "<< dem_pitch*180/M_PI
  //         <<"d_throttle: "<< dem_thr
  //         << std::endl;

  //log actions
  fs_act<< nav_roll*180./M_PI<<" "
        << nav_yaw*180./M_PI<<" "
	<< dem_pitch*180./M_PI<<" "
	<< dem_thr << std::endl;

  //update based on action
  st_end= updater.update(st_start,nav_roll,nav_yaw,dem_pitch,dem_thr,dt);
  //log states
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
}

int NavigatorSim::PropagateWp(UserStructs::PlaneStateSim& st_start,
		 UserStructs::PlaneStateSim& st_end,
		 arma::vec::fixed<2> pt_A,
		 UserStructs::MissionSimPt& pt_target)
{
   //double rho= speed_trim/updater.GetMaxYawRate();
   //double rho= updater.GetMaxSpeed()/updater.GetMaxYawRate();
   double rho= turn_radius;
   
   st_start.CheckConvert();
   pt_target.CheckConvert();
   //std::cout<<"pt_x: "<< pt_target.x
   //         <<" pt_y: "<< pt_target.y
   //	    <<" pt_alt: "<< pt_target.alt
   //	    << std::endl;
   
   //double x1,y1;
   //Utils::ToUTM(pt_A(1),pt_A(0),x1,y1);
   //std::cout<<"angle: "
   //         << atan2(pt_target.x-st_start.x,pt_target.y-st_start.y)*180/M_PI
   //	    << std::endl;

   double yaw1= atan2(pt_target.y-st_start.y, pt_target.x-st_start.x);
   double Ld= Utils::DubinsLength3D(st_start.x,st_start.y,st_start.z,
                              M_PI/2-st_start.yaw,
			      pt_target.x,pt_target.y,pt_target.alt,yaw1,
			      rho);
   //std::cout<< "Ld: "<< Ld << std::endl;
   //update util reached
   UserStructs::PlaneStateSim st_now = st_start;
   UserStructs::PlaneStateSim st_next;
   double length=0.;//length travelled
   int result;//0: arrived by closeness, 1: arrived by length
   int count=0;
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
     //std::cout<<"st_now.z: "<< st_now.z<< std::endl;
     //std::cout<<"count: "<< ++count<< std::endl;
     PropagateStep(st_now,st_next,pt_A,pt_target);

     if(pt_target.SeeArrive(st_next.x,st_next.y,st_next.z) ){
       result= 0;
       break;
     }//if ends

     length+= sqrt( pow(st_now.x-st_next.x,2)
	          + pow(st_now.y-st_next.y,2)
		  + pow(st_now.z-st_next.z,2)
	 );
     //fs_state <<"length/Ld: "<< length/Ld<< std::endl;

     if(length >1.1*Ld){
       result= 1;
       break;
     }
     //std::cout<<"st_next.z: "<< st_next.z<< std::endl;
     st_now= st_next;
     //std::cout<<"*******************************"<< std::endl;
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
   //double rho= speed_trim/updater.GetMaxYawRate();
   //double rho= updater.GetMaxSpeed()/updater.GetMaxYawRate();
   double rho= turn_radius;
   
   st_start.CheckConvert();
   pt_target.CheckConvert();
   //std::cout<<"pt_x: "<< pt_target.x
//	    <<" pt_y: "<< pt_target.y
    //    <<" pt_alt: "<< pt_target.alt
    //    << std::endl;
   
   //double x1,y1;
   //Utils::ToUTM(pt_A(1),pt_A(0),x1,y1);
   //std::cout<<"angle: "
//	    << atan2(pt_target.x-st_start.x,pt_target.y-st_start.y)*180/M_PI
    //    << std::endl;

   double yaw1= atan2(pt_target.y-st_start.y, pt_target.x-st_start.x);
   double Ld= Utils::DubinsLength3D(st_start.x,st_start.y,st_start.z,
			      M_PI/2-st_start.yaw,
			      pt_target.x,pt_target.y,pt_target.alt,yaw1,
			      rho);
   //std::cout<< "Ld: "<< Ld << std::endl;
   //update util reached
   UserStructs::PlaneStateSim st_now = st_start;
   UserStructs::PlaneStateSim st_next;
   length=0.;//length travelled
   int result;//-1:collision, 0: arrived by closeness, 1: arrived by length
   int count=0;
   //loop
   while(1){
     //std::cout<<"st_now.z: "<< st_now.z<< std::endl;
     //std::cout<<"count: "<< ++count<< std::endl;
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
     //std::cout<<"st_next.z: "<< st_next.z<< std::endl;
     st_now= st_next;
     //std::cout<<"*******************************"<< std::endl;
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
    //std::cout<<"PropWpCheck2 print"<< std::endl;
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
    //std::cout<<"MaxSpeed: "<< updater.GetMaxSpeed()<<" "
    //        <<"MaxYawRate: "<< updater.GetMaxYawRate()<<" "
    //	    << std::endl;

    //double rho= updater.GetMaxSpeed()/updater.GetMaxYawRate();
    double rho= turn_radius;
    st_start.CheckConvert();
    pt_target.CheckConvert();
    
    double yaw1= atan2(pt_target.y-st_start.y, pt_target.x-st_start.x);
    double Ld= Utils::DubinsLength2(st_start.x,st_start.y,st_start.z,
			       M_PI/2-st_start.yaw,
			       pt_target.x,pt_target.y,pt_target.alt,yaw1,
			       rho,GetMaxPitch() );
    double len_straight= sqrt( pow(pt_target.x-st_start.x,2)
	                    + pow(pt_target.y-st_start.y,2)
			    + pow(pt_target.alt-st_start.z,2)
	);
    //std::cout<<"length straight: "<< len_straight<< std::endl;
    //std::cout<< "Ld: "<< Ld << std::endl;
    //update util reached
    UserStructs::PlaneStateSim st_now = st_start;
    //std::cout<<"st_now: "
     //       << st_now.x<<" "
    //	<< st_now.y<<" "
    //	<< st_now.z<< std::endl;
    UserStructs::PlaneStateSim st_next;
    length=0.;//length travelled
    //std::cout<<"length: "<< length<< std::endl;
    int result=-2;//-1:collision, 0: arrived by closeness, 1: arrived by length
    int count=0;
    int Nec= 0;
    //loop
    while(1){
      //std::cout<<"st_now.z: "<< st_now.z<< std::endl;
      //std::cout<<"count: "<< ++count<< std::endl;
      PropagateStep(st_now,st_next,pt_A,pt_target);
      //std::cout<<"st_next: "
       //     << st_next.x<<" "
        //<< st_next.y<<" "
        //<< st_next.z<< std::endl;

      //states_rec.push_back(st_next);

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
      //std::cout<<"length: "<< length << std::endl;

      if( (int)(length/check_step) > Nec )
      {
	//check for obstacles
	for(int i=0;i!= obstacles.size();++i){

	   if(Utils::CheckCollision(st_next,obstacles[i])==1){
	     result = -1;
         //std::cout<<"check:collsion with obstacles"<<std::endl;
	     break;
	   }

	}
	//check for spacelimit
	if(result!= -1 && !spacelimit.TellIn(st_next.x,st_next.y,st_next.z) ){
      //std::cout<<"check: out geofence"<< std::endl;
	  result = -1;
	}
       // std::cout<<"Nec: "<< Nec <<" "<< Nec%N_inter<<std::endl;
	if(result!=-1 && Nec % N_inter==0) {
      //std::cout<<"instert"<< std::endl;
	  states_part_rec.push_back(UserStructs::StateNode(st_next,length));
	}
	if(result== -1) break;
      }

      Nec= (int)(length/check_step); 
            
      //fs_state <<"length/Ld: "<< length/Ld<< std::endl;
     // std::cout<<"length/Ld: "<< 1.0*length/Ld<< std::endl;
      states_rec.push_back(st_next);


      if(length >1.1*Ld){
	result= 1;
	break;
      }
      //std::cout<<"st_next.z: "<< st_next.z<< std::endl;
     // st_now= st_next;
      std::cout<<"*******************************"<< std::endl;
    }//while ends
    //std::cout<<"states_part_rec size: "<< states_part_rec.size()<<std::endl;
    //std::cout<<"PropWpCheck2 print ends"<< std::endl;
    st_end= st_next;
    return result;

}//PropWpCheck2 ends

void NavigatorSim::CopyStatesRec(std::vector<UserStructs::PlaneStateSim>& copy_rec)
{
    copy_rec= states_rec;
}

void NavigatorSim::CopyStatePart(std::vector<UserStructs::StateNode>& copy_rec)
{
    copy_rec= states_part_rec;
}

}//namespace ends
