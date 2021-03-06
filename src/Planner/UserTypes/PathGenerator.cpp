#include "PathGenerator.hpp"
//usertypes
#include "Planner/Utils/NotInRadius.h"
#include "OtherLibs/Dubins2D/dubins.h"
#include "common/Utils/GeoUtils.h"
#include "Planner/UserTypes/Sampler/Sampler.hpp"
//std
#include <stdexcept>
#include <fstream>
#include <algorithm>
#include "armadillo"

namespace UasCode{
  //constructor
  PathGenerator::PathGenerator()
  {
    if_start_set= false;
    if_goal_set= false;
    //if_goal_reach= false;
    if_sampler_set= false;
    if_sampler_para_set= false;
    if_spacelimit_set= false;
    if_in_ros= false;
    if_for_plot= false;
    if_limit_reach= false;

    sec_count= 0.; 
    t_limit= 5.;
  }//PathGenerator() ends
  
  //destructor
  PathGenerator::~PathGenerator()
  {
    if(sampler_pt) delete sampler_pt;
  }
  
  //set sampler pointer
  void PathGenerator::SetSampler(UserTypes::Sampler* _sampler_pt)
  {
    this->sampler_pt= _sampler_pt;
    if_sampler_set= true;
  }
  //set space limit, including altitude limit and geofence
  void PathGenerator::SetSpaceLimit(UserStructs::SpaceLimit _limit)
  {
    this->spacelimit= _limit;
    if_spacelimit_set= true;
  }
  //set obstacles
  void PathGenerator::SetObs(const std::vector<UserStructs::obstacle3D>& _obs3ds)
  {
    this->obs3ds= _obs3ds;
  }
  //set N_inter for navigator
  void PathGenerator::SetNinter(const int _N)
  {
    this->navigator.SetInserInterv(_N);
  }
  //set the start state
  void PathGenerator::SetInitState(UserStructs::PlaneStateSim& _st)
  {
    this->st_start= _st; 
    if_start_set= true;
  }
  //set the intermediate state
  void PathGenerator::SetInterState(UserStructs::PlaneStateSim& _st)
  {
    this->st_inter= _st;
  }
  //set the goal waypoint
  void PathGenerator::SetGoalWp(UserStructs::MissionSimPt& _pt)
  {
    this->goal_wp= _pt; 
    if_goal_set= true;
  }
  //set sample method
  void PathGenerator::SetSampleMethod(int _method)
  {
    CheckSamplerSet();
    sampler_pt->SetSampleMethod(_method);
  }
  //update the parameters for the state updater in the navigator
  void PathGenerator::NavUpdaterParams(double _Tmax,double _mpitch_rate,double _myaw_rate,double _Muav, double _max_speed,double _min_speed,double _max_pitch,double _min_pitch)
  {
    navigator.UpdaterSetParams(_Tmax,_mpitch_rate,_myaw_rate,_Muav,
      _max_speed,_min_speed,_max_pitch,_min_pitch); 
    //assign technical parameters
    max_yaw_rate= _myaw_rate;
    max_pitch= _max_pitch;
    max_speed= _max_speed;
  }//NavUpdaterParams ends

  //tecs in navgiator load parameters
  void PathGenerator::NavTecsReadParams(const char* filename)
  {
    navigator.TECsReadParams(filename);
  }//NavTecsReadParams

  //roll limit set for L1_Controller in navigator
  void PathGenerator::NavL1SetRollLim(double _lim)
  {
    navigator.L1SetRollLim(_lim);
  }//L1SetRollLim

  //set dt for the navigator
  void PathGenerator::NavSetDt(double _dt)
  {
    navigator.SetDt(_dt);
    this->dt= _dt;
  }//NavSetDt

  //set speed_trim for navigator
  void PathGenerator::NavSetSpeedTrim(double _trim)
  {
    navigator.SetSpeedTrim(_trim);
  }

  //set sampler parameters based on st_start and goal_wp
  void PathGenerator::SetSampleParas()
  {
    CheckStartSet();
    CheckGoalSet();
    CheckSamplerSet();

    sampler_pt->SetSampleMethod(0);
    sampler_pt->SetParams(st_start,goal_wp,this->max_pitch*0.1);
    if_sampler_para_set= true;
  }

  //sample a wp
  void PathGenerator::SamplePt()
  {//sample, check radius limit, check pitch limit
    double x_root= st_start.x;
    double y_root= st_start.y;
    double z_root= st_start.z;
    double yaw_root= M_PI/2- st_start.yaw;
    std::cout<< "sample root "
             << x_root<< " "
	     << y_root<< " "
	     << z_root<< " "
	     << yaw_root*180./M_PI << std::endl;
    double x_a,y_a,z_a,the_a;
    double rho= max_speed/max_yaw_rate;
    std::cout<<"rho: "<< rho<< std::endl;

    while(1)
    {
      sampler_pt->GetSample(x_a,y_a,z_a,st_start,goal_wp);
      std::cout<<"sample check "
	 <<x_a<<" "<<y_a<<" "<<z_a<<" "<< std::endl;
      //check
      //if in radius range
      bool if_radius= Utils::NotInRadius(x_root,y_root,yaw_root,x_a,y_a,rho);
      if(!if_radius) {
        std::cout<<"in radius"<< std::endl;
	continue;
      }
      //if pitch ok
      DubinsPath path;
      the_a= atan2(y_a-y_root,x_a-x_root);
      double q0[]={x_root,y_root,yaw_root};
      double q1[]={x_a,y_a,the_a};
      dubins_init(q0,q1,rho,&path);

      double length= dubins_path_length(&path);
      double h= fabs(z_a-z_root);
      double gamma_d= atan2(h,length);
      bool if_ga= (gamma_d<= max_pitch );
      if(!if_ga) 
      {	
	std::cout<<"ga too large"<< std::endl;
	continue;
      }//if out of geo fence
      bool if_in= spacelimit.TellIn(x_a,y_a,z_a);
      if(!if_in) std::cout<<"out fence"<<std::endl;
      //if(!if_in) {;}
      else break;
    }//while ends
    //assign sample wp
    double yaw_wp= Utils::_wrap_pi(M_PI/2-the_a);
    float r_wp= std::max(100., max_speed*2*dt);
    //the last 3 number doesn't matter because we use r for arrival justification
    //we have to adjust the wp arrival part the the firmware
    sample_wp= UserStructs::MissionSimPt(0,0,z_a,yaw_wp,r_wp,x_a,y_a,200,100,50);
    sample_wp.CheckConvert();
  }

  //that is a free function for comparison
  bool WpCompFunc(UserStructs::WpLength wpl1, UserStructs::WpLength wpl2)
  {
    return (wpl1.length < wpl2.length);
  }
   
  //add more possible paths
  //return the number of possible paths
  int PathGenerator::AddPaths()
  {
    //about time limit
    if(!if_in_ros)
    {
      ros::Time::init();
      t_start = ros::Time::now();//timing start point 
    }
    std::cout<<"path adding begins"<< std::endl;
    int sample_count = 0;//effective sample
    int sample_raw = 0;//raw samples
    sec_count = ros::Time::now().toSec()-t_start.toSec();
    //for log option 
    std::ofstream fs_sample("sample.txt");
    //clear previous path
    wp_lengths.clear();
    //the loop
    //just for ISER paper, just add goal check temperarily
    double length= 0;
    UserStructs::PlaneStateSim st_end;
    arma::vec::fixed<2> pt_A;
    pt_A << st_start.lat << st_start.lon;

    int result1= navigator.PropWpCheck2(st_start,
	  st_end,
	  pt_A,
	  goal_wp,
	  obs3ds,
	  spacelimit,
	  length,0);
    if(result1!=-1) 
     wp_lengths.push_back(UserStructs::WpLength(goal_wp,length,result1)); 

    while(1)
    {
    //sample a wp
      SamplePt(); 
      
      if(if_for_plot){
        fs_sample << sample_wp.x<<" "
	          << sample_wp.y<<" "
	          << sample_wp.alt<<" "
		  << std::endl;
      }//if_for_plot ends
      
      ++sample_raw;
    //check:
    //first check from start to sample_wp;
      length = 0; 
      int result1= navigator.PropWpCheck2(st_start,
                     st_end,
                     pt_A,
                     sample_wp,
                     obs3ds,
		     spacelimit,
                     length,
		     0);
      std::cout<<"result1: "<< result1<< std::endl;
      double count1= ros::Time::now().toSec()-t_start.toSec();
      std::cout<<"count1: "<< count1<< std::endl;

      if(result1!= -1)
      {//the first section is collision free
	++sample_count;
        //copy the state_rec
	temp_part_rec.clear();
	navigator.CopyStatePart(temp_part_rec);
        temp_part_rec.push_back(UserStructs::StateNode(st_end,length));
	//check some states in temp_rec, their reachability to the
	std::cout<<"temp_part_rec size: "<< temp_part_rec.size()<<std::endl;
	//goal wp
	for(int i=0;i!=temp_part_rec.size();++i)
	{
	  UserStructs::PlaneStateSim st_init= temp_part_rec[i].state;
	  pt_A<< st_init.lat << st_init.lon;
	  double length1= temp_part_rec[i].length;

	  length= 0;
          int result2= navigator.PropWpCheck2(st_init,
	               st_end,
		       pt_A,
		       goal_wp,
		       obs3ds,
		       spacelimit,
		       length,
		       1);
	  std::cout<<"result2: "<< result2<< std::endl; 
	  if(result2!=-1){
	    float r_wp= std::max(100., max_speed*2*dt);
	    UserStructs::MissionSimPt new_wp
	     = UserStructs::MissionSimPt(0,0,st_init.z,st_init.yaw,r_wp,st_init.x,st_init.y,200,100,50);
	    new_wp.CheckConvert();
	    wp_lengths.push_back(UserStructs::WpLength(new_wp,length+length1,result2)); 
	  }//if result2!=-1 ends

          //timer
	  if(!if_limit_reach){
	    sec_count=ros::Time::now().toSec()-t_start.toSec();
	    std::cout<<"sec_count1: "<<sec_count<<std::endl;
	    if(sec_count>= t_limit)
	    {
	      if_limit_reach= true;
	      std::cout<<"stop add intermediate path"<< std::endl;
	      break;
	    }
	  }//

	}//for int i ends
      }//if result1!= -1 ends
    
      //timer
      if(!if_limit_reach){
        sec_count=ros::Time::now().toSec()-t_start.toSec();
        std::cout<<"sec_count2: "<<sec_count<<std::endl;
	if(sec_count>= t_limit)
	{
          if_limit_reach= true;
	  std::cout<<"stop while loop"<< std::endl;
	}
      }//

      if(if_limit_reach) break;
    }//while ends
    //if(!wp_lengths.empty() ) return true;
    //return false;
    return wp_lengths.size();

  }//AddPath function ends

  
  UserStructs::MissionSimPt PathGenerator::GetInterWp()
  { //this function may not be used eventually
    //AddPaths();
    //sort according to total length
    //std::sort(wp_lengths.begin(),wp_lengths.end(),WpCompFunc);
    //select the one with the least length
    //return wp_lengths[0].wp;
    return inter_wp;
  }//GetInterWp ends

  UserStructs::PlaneStateSim PathGenerator::GetInterState()
  {
    return this->st_inter;
  } 

  bool PathGenerator::PathCheckSingle(UserStructs::PlaneStateSim st_current)
  {//true:bad, false: good
    arma::vec::fixed<2> pt_A;
    pt_A << st_current.lat << st_current.lon;
    UserStructs::PlaneStateSim st_end;
    double length= 0.;

    int result1= navigator.PropWpCheck2(st_current,
		  st_end,
		  pt_A,
		  inter_wp,
		  obs3ds,
		  spacelimit,
		  length,
		  0);
    if(result1== -1) return true;
    //check next section
    pt_A<< inter_wp.lat << inter_wp.lon;

    result1= navigator.PropWpCheck2(st_end,
	          st_end,
                  pt_A,
		  goal_wp,
                  obs3ds,
		  spacelimit,
		  length,
		  1);
     if(result1== -1) return true;
     return false;
  }//PathCheckSingle ends

  bool PathGenerator::PathCheckRepeat(UserStructs::PlaneStateSim st_current)
  {
    //following AddPath()
    if(wp_lengths.empty() ) return false;
    //check if the path is still collision free
    bool if_colli= false, if_path= false;
    ros::Time t1= ros::Time::now();
    //sort according to total length
    std::sort(wp_lengths.begin(),wp_lengths.end(),WpCompFunc);
    int count= 0;
    //overhead
    UserStructs::PlaneStateSim st_end;
    double length= 0;

    while(1)
    {
      //TIME LIMIT FOR CHECK REPEAT
      if(ros::Time::now()- t1 >= ros::Duration(0.1) )
      {
        std::cout<<"PathCheckRepeat time up"<< std::endl;
	//if_path= false;
	break;
      }
      
      if(count== wp_lengths.size()){
        break;
      }

      UserStructs::MissionSimPt wp= wp_lengths[count].wp;
      //check from current to intermediate waypoint
      arma::vec::fixed<2> pt_A;
      pt_A << st_current.lat << st_current.lon;

      int result1= navigator.PropWpCheck2(st_current,
		  st_end,
		  pt_A,
		  wp,
		  obs3ds,
		  spacelimit,
		  length,
		  0);
      std::cout<<"current recheck re1: "<< result1<< std::endl;
      if(result1== -1){
        ++count;
	continue;
      }
      else{//for the final log
        navigator.CopyStatesRec(temp_rec);
	total_rec= temp_rec;
      }

      SetInterState(st_end);
      std::cout<<"inter state: "
	       <<"st_end.x: "<< st_end.x<<" "
	       <<"st_end.y: "<< st_end.y<<" "
	       <<"st_end.z: "<< st_end.z<<std::endl;
      //check from the intermediate waypoint to the goal waypoint
      pt_A<< wp.lat << wp.lon;

      result1= navigator.PropWpCheck2(st_end,
	          st_end,
                  pt_A,
		  goal_wp,
                  obs3ds,
		  spacelimit,
		  length,
		  1);
      std::cout<<"current recheck re2: "<< result1<< std::endl;

      if(result1==-1){
        ++count;
	continue;
      }
      else{
	//for the final log
        navigator.CopyStatesRec(temp_rec);
        total_rec.insert(total_rec.end(),temp_rec.begin(),temp_rec.end());	
        //final intermediate waypoint
	inter_wp= wp;
	break;
      }
    }//while ends
    if(count< wp_lengths.size() ){
        return true; 
    }
    return false;
  }//PathCheckRepeat ends

  void PathGenerator::PrintPath(const char* filename){
    std::ofstream fs(filename);
    if(fs.is_open() ){
      std::cout<<"pathgen printpath file open"<< std::endl;
      for(int i=0;i!=total_rec.size();++i){
	UserStructs::PlaneStateSim st_end= total_rec[i];
        fs<< st_end.t<<" "
          << st_end.x<<" "
	  << std::setprecision(7)<< st_end.y<<" "
	  << st_end.lat<<" "
	  << st_end.lon<<" "
	  << st_end.z<<" "
	  << st_end.speed<<" "
	  << st_end.yaw/M_PI*180.<<" "
	  << st_end.pitch/M_PI*180.<<" "
	  << st_end.ax<<" "<< st_end.ay<<" "<< st_end.az<<" "
	  << std::endl;
      }//for i ends
    }//if is_open ends

    fs.close();
  }

  //check set flags
  void PathGenerator::CheckStartSet()
  {
    if(!if_start_set){
      try{ throw std::runtime_error("start not set"); }
      catch(std::runtime_error& e){
        std::cout<< "caught a runtime_error execption: "
	   << e.what() << std::endl;
      }
    }
  }

  void PathGenerator::CheckGoalSet()
  {
    if(!if_goal_set){
      try{ throw std::runtime_error("goal not set"); }
      catch(std::runtime_error& e){
	std::cout<< "caught a runtime_error execption: "
	   << e.what() << std::endl;
      }
    }  
  }//CheckGoalSet ends

  void PathGenerator::CheckSamplerSet()
  {
    if(!if_sampler_set){
      try{throw std::runtime_error("sampler not set"); }
      catch(std::runtime_error& e){
        std::cout<<"caught a runtime_error execption: "
	  << e.what() << std::endl;
      }
    }//if 
  }

  void PathGenerator::CheckSamplerParaSet()
  {
    if(!if_sampler_para_set){
      try{ throw std::runtime_error("sampler_para not set"); }
      catch(std::runtime_error& e){
	std::cout<< "caught a runtime_error execption: "
	   << e.what() << std::endl;
      }
    } 
  }//CheckSamplerParaSet ends

  void PathGenerator::CheckSpaceLimitSet()
  {
    if(!if_spacelimit_set){
      try{ throw std::runtime_error("spacelimit not set"); }
      catch(std::runtime_error& e){
	std::cout<< "caught a runtime_error execption: "
	   << e.what() << std::endl;
      }
    }  
}//CheckGoalSet ends


}//namespace ends
