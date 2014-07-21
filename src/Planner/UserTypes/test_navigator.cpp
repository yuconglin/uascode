#include "NavigatorSim.hpp"
#include "common/UserStructs/constants.h"
#include "common/Utils/GetTimeUTC.h"
#include "Planner/UserStructs/PlaneStateSim.h"
//other
#include "armadillo"
#include <fstream>

using namespace UasCode;

int main(int argc,char** argv)
{
  NavigatorSim navigator("fs_action.txt","fs_state.txt");
  //parameters
  double _Tmax= 12.49*CONSTANT_G;
  //double _Tmax= 5*CONSTANT_G;
  //double _Muav= 29.2; //kg
  double _Muav= 10;
  double myaw_rate= 20./180*M_PI;
  double mpitch_rate= 10./180*M_PI;
  double _max_speed= 30; //m/s
  double _min_speed= 10; //m/s
  double _max_pitch= 25./180*M_PI;
  double _min_pitch= -20./180*M_PI;

  double dt= 1.0;
  //double _speed_trim= 0.5*(_max_speed+_min_speed);
  double _speed_trim= _max_speed;

  navigator.UpdaterSetParams(_Tmax,mpitch_rate,myaw_rate,_Muav,
      _max_speed,_min_speed,_max_pitch,_min_pitch);

  navigator.TECsReadParams("parameters_sitl.txt");
  //navigator.TECsReadParams("parameters.txt");
  navigator.L1SetRollLim(40./180*M_PI);
  navigator.SetDt(dt);
  navigator.SetSpeedTrim(_speed_trim);
  //navigator.EnableAirspd();
  /****read start state****/
  std::ifstream start_file("/home/yucong/ros_workspace/uascode/bin/sitl_state1.txt");
  double t_now,lon_f,lat_f,hgt_f,speed_f,x_f,y_f,hd_f,roll_f,pitch_f,yaw_f,ax_f,ay_f,az_f,dvz_f;
  for(int i=0;i!=1;++i){
    start_file >> t_now
            >> lon_f
            >> lat_f
            >> hgt_f
            >> speed_f
            >> x_f >> y_f >> hd_f
            >> roll_f >> pitch_f >> yaw_f
            >> ax_f >> ay_f >> az_f >> dvz_f;
  }
  /****read end state****/
  //test
  arma::vec::fixed<2> pt_A, pt_B;
//1405185947.83948 -111.92720 33.42759 734.04000 26.47640 413803.76236 3699075.94730 13.00000 -0.73547 0.23162 0.22899 -100.00000 547.00000 -707.00000 -16.66667
  double lon= lon_f, lat= lat_f;

  double lat1= 33.421964, lon1= -111.940082;//wp 1
  //double lat1= 33.414728, lon1= -111.939738;//wp 2
  pt_A << 33.427559 << -111.926697; //here needs to be changed
  pt_B << lat1 << lon1;

  double hgt= hgt_f;
  double spd= speed_f;
  double t= t_now;
  double yaw= yaw_f;
  double pitch= pitch_f;
  double alt_B= 615;
  double ax= ax_f/1000*CONSTANT_G, ay= ay_f/1000*CONSTANT_G;
  //double az= -(1.0-793.00000/1000)*CONSTANT_G;
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
