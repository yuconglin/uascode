#include "PathGenerator.hpp"
//usertypes
#include "Planner/Utils/NotInRadius.h"
#include "OtherLibs/Dubins2D/dubins.h"
#include "common/Utils/GeoUtils.h"
#include "common/Utils/YcLogger.h"
#include "Planner/UserTypes/Sampler/Sampler.hpp"
//std
#include <stdexcept>
#include <fstream>
#include <algorithm>
#include "armadillo"

namespace{
Utils::LoggerPtr s_logger(Utils::getLogger("uascode.PathGenerator.YcLogger"));
}

namespace UasCode{
  //constructor
  PathGenerator::PathGenerator():sample_method(0)
  {
    if_start_set= false;
    if_start_wp_set= false;
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

  //set obstacles but multiplied by a thres_ratio
  void PathGenerator::SetObsThres(const std::vector<UserStructs::obstacle3D> &_obs3ds, const double _thres){
     this->obs3ds = _obs3ds;
     for(int i=0;i!= obs3ds.size();++i){
            obs3ds[i].r *= _thres;
     }
  }

  //set N_inter for navigator
  void PathGenerator::SetNinter(const int _N)
  {
    this->navigator.SetInserInterv(_N);
  }
  //set the start state
  void PathGenerator::SetInitState(UserStructs::PlaneStateSim _st)
  {
    this->st_start= _st; 

    UASLOG(s_logger,LL_DEBUG,"st_start z: "<< st_start.z);

    if_start_set= true;
  }

  //set sampling start
  void PathGenerator::SetSampleStart(double _x_start,double _y_start,double _z_start)
  {
      this->xs_start= _x_start;
      this->ys_start= _y_start;
      this->zs_start= _z_start;
  }

  //set the intermediate state
  void PathGenerator::SetInterState(UserStructs::PlaneStateSim& _st)
  {
    this->st_inter= _st;
  }
  //set the start waypoint
  void PathGenerator::SetStartWp(UserStructs::MissionSimPt &_pt)
  {
    this->start_wp= _pt;
    if_start_wp_set= true;
  }
  //set the begin waypoint
  void PathGenerator::SetBeginWp(UserStructs::MissionSimPt& _pt)
  {
     this->begin_wp= _pt;
  }

  //set the goal waypoint
  void PathGenerator::SetGoalWp(UserStructs::MissionSimPt& _pt)
  {
    this->goal_wp= _pt; 

    UASLOG(s_logger,LL_DEBUG,"set goal: "
           << goal_wp.lat << " "
           << goal_wp.lon << " "
           << goal_wp.alt);

    if_goal_set= true;
  }
  //set in-between wps
  void PathGenerator::SetBetweenWps(const std::vector<UserStructs::MissionSimPt> _wpoints)
  {
    this->WpInBetweens= _wpoints;
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
    //sampler_pt->SetParams(st_start,goal_wp,this->max_pitch*0.1);
    sampler_pt->SetParams2(xs_start,ys_start,zs_start,goal_wp,this->max_pitch*0.1);
    if_sampler_para_set= true;
  }

  double PathGenerator::GetTurnRadius()
  {
    return max_speed/max_yaw_rate;
  }

  //sample a wp
  void PathGenerator::SamplePt()
  {//sample, check radius limit, check pitch limit

    double x_a,y_a,z_a,the_a;
    double rho= max_speed/max_yaw_rate;
    //UASLOG(s_logger,LL_DEBUG,"rho: "<< rho);
    double dis_total= sqrt(pow(xs_start-goal_wp.x,2)+pow(ys_start-goal_wp.y,2));

    while(1)
    {
      sampler_pt->GetSample2(x_a,y_a,z_a,xs_start,ys_start,zs_start,goal_wp);
      //if pitch ok
      DubinsPath path;
      the_a= atan2(y_a-ys_start,x_a-xs_start);
      double q0[]={xs_start,ys_start,this->yaw_root};
      double q1[]={x_a,y_a,the_a};
      dubins_init(q0,q1,rho,&path);

      double length= dubins_path_length(&path);
      double h= fabs(z_a-zs_start);
      double gamma_d= atan2(h,length);
      bool if_ga= (gamma_d<= max_pitch );

      if(!if_ga)
      {	
          UASLOG(s_logger,LL_DEBUG,"ga too large");
          continue;
      }//if out of geo fence

      bool if_in= spacelimit.TellIn(x_a,y_a,z_a);

      if(!if_in){
          UASLOG(s_logger,LL_DEBUG,"out fence");
          continue;
      }

      double dis_sample= std::sqrt(pow(x_a-xs_start,2)+pow(y_a-ys_start,2));
      if(dis_sample > dis_total){
          //UASLOG(s_logger,LL_DEBUG,"too long sample");
      }
      else break;
    }//while ends
    //UASLOG(s_logger,LL_DEBUG,"sample ok");
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
    }

    if_limit_reach= false;
    t_start = ros::Time::now();//timing start point

    UASLOG(s_logger,LL_DEBUG,"path adding begins");
    int sample_count = 0;//effective sample
    int sample_raw = 0;//raw samples
    sec_count = ros::Time::now().toSec()-t_start.toSec();
    //for log option 
    std::ofstream fs_sample("sample.txt");
    //clear previous path
    wp_lengths.clear();

    UserStructs::PlaneStateSim st_ps= st_start, st_second;
    for(int i=0;i!= WpInBetweens.size();++i)
    {
       arma::vec::fixed<2> pt_temp;
       if(i==0){
           pt_temp << begin_wp.lat << begin_wp.lon;
       }
       else{
           pt_temp << WpInBetweens[i-1].lat << WpInBetweens[i-1].lon;
       }
       UASLOG(s_logger,LL_DEBUG,"wp in between:"<< WpInBetweens[i].lat << " "<< WpInBetweens[i].lon);
       navigator.PropagateWp(st_ps,st_second,pt_temp,WpInBetweens[i]);
       st_ps= st_second;
    }

    double x_ps, y_ps;
    Utils::ToUTM(st_ps.lon,st_ps.lat,x_ps,y_ps);
    UASLOG(s_logger,LL_DEBUG,"st_ps: "<< st_ps.lat<<" "
           << st_ps.lon<< " "<< x_ps<<" "<< y_ps <<" "
           << "hd:"<< st_ps.yaw*180./M_PI);

    //the loop
    double length= 0;
    UserStructs::PlaneStateSim st_end;
    arma::vec::fixed<2> pt_A;
    pt_A << start_wp.lat << start_wp.lon;
    this->SetYawRootSample(M_PI/2-st_ps.yaw);

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
    //first check from start to each waypoints.

    //then check from start to sample_wp;
      length = 0; 
      int result1= navigator.PropWpCheck2(st_ps,
                                          st_end,
                                          pt_A,
                                          sample_wp,
                                          obs3ds,
                                          spacelimit,
                                          length,
                                          0);

      if(result1!= -1)
      {//the first section is collision free
          //UASLOG(s_logger,LL_DEBUG,"first section collision free");
          ++sample_count;
          //copy the state_rec
          temp_part_rec.clear();
          navigator.CopyStatePart(temp_part_rec);

          temp_part_rec.push_back(UserStructs::StateNode(st_end,length));
          //check some states in temp_rec, their reachability to the
          //UASLOG(s_logger,LL_DEBUG,"temp_part_rec size: "<< temp_part_rec.size() );
          //goal wp
          for(int i=0;i!=temp_part_rec.size();++i)
          {
              UserStructs::PlaneStateSim st_init= temp_part_rec[i].state;
              //pt_A<< st_init.lat << st_init.lon;
              pt_A<< sample_wp.lat << sample_wp.lon;
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
                  if(sec_count>= t_limit)
                  {
                      if_limit_reach= true;
                      break;
                  }
              }//

          }//for int i ends
          //UASLOG(s_logger,LL_DEBUG,"wp_lengths size:"<< wp_lengths.size() );
      }//if result1!= -1 ends
      else{
          //UASLOG(s_logger,LL_DEBUG,"first section collided");
      }
    
      //timer
      if(!if_limit_reach){
        sec_count=ros::Time::now().toSec()-t_start.toSec();

        if(sec_count>= t_limit)
        {
            if_limit_reach= true;
        }//
      }

      if(if_limit_reach){
          UASLOG(s_logger,LL_DEBUG,"stop add intermediate path"
                 <<" "<< "time used: " << sec_count
                 <<" "<< "num of wps: " << wp_lengths.size() );
          UASLOG(s_logger,LL_DEBUG,"path adding ends");
         break;
      }
    }//while ends

    return wp_lengths.size();

  }//AddPath function ends

  
  UserStructs::MissionSimPt PathGenerator::GetInterWp()
  {
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
    ros::Time t1= ros::Time::now();
    //sort according to total length
    std::sort(wp_lengths.begin(),wp_lengths.end(),WpCompFunc);
    int count= 0;
    //overhead
    UserStructs::PlaneStateSim st_end;
    double length= 0;
    bool if_time_up = false;

    UserStructs::PlaneStateSim st_ps= st_current, st_second;

    for(int i=0;i!= WpInBetweens.size();++i)
    {
       arma::vec::fixed<2> pt_temp;
       if(i==0){
           pt_temp << begin_wp.lat << begin_wp.lon;
       }
       else{
           pt_temp << WpInBetweens[i-1].lat << WpInBetweens[i-1].lon;
       }
       UASLOG(s_logger,LL_DEBUG,"wp in between:"<< WpInBetweens[i].lat << " "<< WpInBetweens[i].lon);
       navigator.PropagateWp(st_ps,st_second,pt_temp,WpInBetweens[i]);
       st_ps= st_second;
    }

    while(1)
    {
      //TIME LIMIT FOR CHECK REPEAT
      UASLOG(s_logger,LL_DEBUG,"in PathCheckRepeat");

      if(ros::Time::now()- t1 >= ros::Duration(0.1) )
      {
          UASLOG(s_logger,LL_DEBUG,"PathCheckRepeat time up");
          if_time_up= true;
          break;
      }
      
      if(count== wp_lengths.size()){
          UASLOG(s_logger,LL_DEBUG,"all path check failed");
        break;
      }

      UserStructs::MissionSimPt wp= wp_lengths[count].wp;
      UASLOG(s_logger,LL_DEBUG,"wp:"<< count <<" "<< wp.x <<" "<< wp.y);
      //check from current to intermediate waypoint
      arma::vec::fixed<2> pt_A;
      pt_A << start_wp.lat << start_wp.lon;

      int result1= navigator.PropWpCheck2(st_ps,
                                          st_end,
                                          pt_A,
                                          wp,
                                          obs3ds,
                                          spacelimit,
                                          length,
                                          1);

      if(result1== -1){
          //UASLOG(s_logger,LL_DEBUG,"first section failed");
          ++count;
          continue;
      }
      else{//for the final log
          navigator.CopyStatesRec(temp_rec);
          total_rec= temp_rec;
      }

      SetInterState(st_end);
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

      if(result1==-1){
          UASLOG(s_logger,LL_DEBUG,"second half failed");
          ++count;
          continue;
      }
      else{
	//for the final log
          navigator.CopyStatesRec(temp_rec);
          total_rec.insert(total_rec.end(),temp_rec.begin(),temp_rec.end());
          //final intermediate waypoint
          inter_wp= wp;
          UASLOG(s_logger,LL_DEBUG,"the_s: "<< atan2(inter_wp.y-ys_start,inter_wp.x-xs_start)*180./M_PI );
          UASLOG(s_logger,LL_DEBUG,"wp " << count<< "was selected");
          UASLOG(s_logger,LL_DEBUG,"xs_start:"<< xs_start<<" "<< "ys_start:"<< ys_start);
          UASLOG(s_logger,LL_DEBUG,"x_goal:"<< goal_wp.x<<" "<<"y_goal:"<< goal_wp.y);
          break;
      }
    }//while ends

    if(if_time_up)
        return false;

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

  void PathGenerator::CheckStartWpSet()
  {
      if(!if_start_wp_set){
          try{ throw std::runtime_error("start wp not set"); }
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
