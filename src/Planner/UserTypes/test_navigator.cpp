#include "NavigatorSim.hpp"
#include "common/UserStructs/constants.h"
#include "common/Utils/GetTimeUTC.h"
#include "common/Utils/FindPath.h"
#include "Planner/UserStructs/PlaneStateSim.h"
//other
#include "armadillo"
#include <fstream>

using namespace UasCode;

int main(int argc,char** argv)
{
  NavigatorSim navigator("fs_action.txt","fs_state.txt");
  //parameters
  //double _Tmax= 12.49*CONSTANT_G;
  double _Tmax= 6.79*CONSTANT_G;
  //double _Muav= 29.2; //kg
  double _Muav= 0.453592*13;
  double myaw_rate= 20./180*M_PI;
  double mpitch_rate= 10./180*M_PI;
  double _max_speed= 30.8667; //m/s
  double _min_speed= 10; //m/s
  double _max_pitch= 25./180*M_PI;
  double _min_pitch= -20./180*M_PI;

  double dt= 0.1;
  //double _speed_trim= 0.5*(_max_speed+_min_speed);
  double _speed_trim= _max_speed;
  //the start id of waypoint
  int des_id= atoi(argv[1]);

  navigator.UpdaterSetParams(_Tmax,mpitch_rate,myaw_rate,_Muav,
      _max_speed,_min_speed,_max_pitch,_min_pitch);

  std::string param_file = Utils::FindPath()+"parameters/parameters_sitl.txt";
  navigator.TECsReadParams(param_file.c_str());
  navigator.L1SetRollLim(40./180*M_PI);
  navigator.SetDt(dt);
  navigator.SetSpeedTrim(_speed_trim);
  //navigator.EnableAirspd();

  /****read start state****/
  std::ifstream start_file( (Utils::FindPath()+"/records/sitl_state_large.txt").c_str() );
  double t_now,lon_f,lat_f,hgt_f,speed_f,x_f,y_f,hd_f,roll_f,pitch_f,yaw_f,ax_f,ay_f,az_f,dvz_f;
  int wp_id = -1;

  std::string line;
  while(std::getline(start_file,line)){
      std::istringstream iss(line);
      iss >> t_now
              >> lon_f
              >> lat_f
              >> hgt_f
              >> speed_f
              >> x_f >> y_f >> hd_f
              >> roll_f >> pitch_f >> yaw_f
              >> ax_f >> ay_f >> az_f >> dvz_f >> wp_id;
      if(wp_id == des_id) break;
  }
  /****read end state****/

  /****read all waypoints****/
  std::vector< arma::vec::fixed<3> > wps;
  std::ifstream plan_file("/home/yucong/yucong_codes_git/sitl/ardupilot/Tools/autotest/ap_large.txt");
  int line_count = 0;
  if(plan_file.is_open())
  {
      std::string line;
      while(getline(plan_file,line))
      {
          if(line_count == 0){
              std::string str1, str2, str3;
              std::istringstream iss(line);
              iss >> str1 >> str2 >> str3;
          }
          else{
              std::istringstream iss(line);
              //0 1 0 16 0.00000 0.000000 0.000000 0.000000 33.422036 -111.926263 30 1
              int seq,frame,command,current,autocontinue;
              float param1,param2,param3,param4;
              double lat,lon,alt;

              iss >> seq >> current >> frame >> command
                      >> param1 >> param2 >> param3 >> param4
                      >> lat >> lon >> alt >> autocontinue;

              alt= alt+585;
              arma::vec::fixed<3> wp;
              wp<< lat<< lon<< alt;
              wps.push_back(wp);
          }//if line_count > 0 ends
          ++line_count;
      }//while plan_file ends

  }
  /****read all waypoints ends***/
  //test
  arma::vec::fixed<2> pt_A, pt_B;

  if(des_id == 1){
      pt_A << lat_f << lon_f;
  }
  else{
      arma::vec::fixed<3> wpp= wps[des_id-1];
      pt_A << wpp(0) << wpp(1);
  }
  arma::vec::fixed<3> wpp= wps[des_id];
  pt_B << wpp(0)<< wpp(1);

  std::cout<<"pt_A: "<< pt_A(0)<<" "<< pt_A(1)<<"\n";
  std::cout<<"pt_B: "<< pt_B(0)<<" "<< pt_B(1)<<"\n";

  double lat= lat_f;
  double lon= lon_f;
  double hgt= hgt_f;
  double spd= speed_f;
  double t= t_now;
  double yaw= yaw_f;
  double pitch= pitch_f;
  double alt_B= 615;
  double ax= ax_f/1000*CONSTANT_G, ay= ay_f/1000*CONSTANT_G;

  double az= az_f/1000*CONSTANT_G;

  UserStructs::PlaneStateSim st(t,0,0,lat,lon,hgt,spd,yaw,pitch,ax,ay,az); 
  //target mission point
  UserStructs::MissionSimPt Pt(pt_B(0),pt_B(1),alt_B,yaw,100,0,0,400,300,30);

  //future state
  UserStructs::PlaneStateSim st_end;
  //go one step
  //navigator.PropagateStep(st,st_end,pt_A,Pt);
  //go to the waypoint: between two normal waypoints
  int result= navigator.PropagateWp(st,st_end,pt_A,Pt);
  //go to the first waypoint
  //int result= navigator.PropagateWpInit(st,st_end,Pt);
  std::cout<< "result: "<< result<< std::endl;
}
