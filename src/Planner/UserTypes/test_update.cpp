#include "StateUpdateSim.hpp"
#include "common/UserStructs/constants.h"
#include "common/Utils/GetTimeUTC.h"
#include "Planner/UserStructs/PlaneStateSim.h"
//user types
#include "Planner/UserTypes/L1ControlSim.hpp"
#include "Planner/UserTypes/TECsSim.hpp"
//std
#include <cmath>
#include "armadillo"
#include <sstream>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>

using namespace UasCode;

int main(int argc,char** argv)
{
  double _Tmax= 12.49*CONSTANT_G;
  double _vertAccLim= 7.;
  double _Muav= 29.2; //kg
  double _max_speed= 35; //m/s
  double _min_speed= 20; //m/s
  double _max_pitch= 20./180*M_PI;
  double _min_pitch= -20./180*M_PI;
  //
  double hgt= 1000;
  double spd= 25;//m/s
  double Lc= 2;//meter, the distance between cg to rudder 
  double myaw_rate= 10./180*M_PI;
  std::cout<<"myaw_rate: "<< myaw_rate<< std::endl;
  double mpitch_rate= 5./180*M_PI;
  double dt=1;
  //state updater
  StateUpdateSim updater(_Tmax,mpitch_rate,myaw_rate,_Muav,
      _max_speed,_min_speed,_max_pitch,_min_pitch); 

  //state
  //double lat=33.415828,lon=-111.934800;
  double lat=33.433477,lon=-111.901494;
  double t= Utils::GetTimeUTC();
  double yaw= 90./180*M_PI;
  double pitch=0.;
  UserStructs::PlaneStateSim st(t,0,0,lat,lon,hgt,spd,yaw,pitch,0,0,0);
  //l1 controller 
  L1ControlSim l1_control;
  l1_control.set_l1_roll_limit(10./180*M_PI); 
  arma::vec::fixed<2> pt_A;
  arma::vec::fixed<2> pt_B;
  arma::vec::fixed<2> curr_posi;
  arma::vec::fixed<2> gnd_speed;

  pt_A<<33.385903<<-111.965008;
  pt_B<<33.387336<<-111.837292;
  curr_posi<< lat<< lon;
  double gnd_e= spd*cos(pitch)*cos(M_PI/2-yaw);
  double gnd_n= spd*cos(pitch)*sin(M_PI/2-yaw);
  gnd_speed<<gnd_n << gnd_e;
  //std::cout<<"pt_A: "<<pt_A(0)<<" "<<pt_A(1)<< std::endl;
  //std::cout<<"pt_B: "<< pt_B(0)<<" "<<pt_B(1)<< std::endl;
  //std::cout<<"curr_posi: "<< curr_posi(0)<<" "<< curr_posi(1)<< std::endl;
  //std::cout<<"gnd_speed: "<< gnd_speed(0)<<" "<< gnd_speed(1)<< std::endl;
  //navigate_waypoints
  l1_control.navigate_waypoints(pt_A,pt_B,curr_posi,gnd_speed);
  //see caculated target roll and bearing
  double nav_roll= l1_control.nav_roll();
  double nav_yaw= l1_control.nav_bearing(); 
  std::cout<<"d_roll: "<< nav_roll/M_PI*180.<<" d_bearing: "<< nav_yaw/M_PI*180.<< std::endl;

  //tecs
  UasCode::TECsSim tecs;
  tecs._load_param_from_file("/home/yucong/yucong_codes_git/parameters.txt");
  //SET dt
//  tecs.set_dt(dt);
  //test update_50hz
  arma::vec::fixed<3> accel;
  accel << st.ax << st.ay << st.az;
  tecs.update_50hz(hgt,accel);
  //
  float eas2tas=1.0;
  float speed_trim= 30.;
  float speed= 25;
  //float pitch= 0.;
  bool climbOutDem= false;
  tecs.update_pitch_throttle(pitch, hgt, hgt+100, speed_trim, speed, eas2tas, false);
    //get
  double dem_pitch= tecs.get_pitch_demand();
  double dem_thr= tecs.get_throttle_demand();
  std::cout<<"pitch: "<< dem_pitch*180/M_PI
           <<" throttle: "<< dem_thr
	   << std::endl;
  //update in action 
  UserStructs::PlaneStateSim st1= updater.update(st,nav_roll,nav_yaw,dem_pitch,dem_thr,dt);
  //print previous and updated state:
  std::cout<<"pre:"
      <<" t: "<< st.t
      <<" x: "<< st.x
      <<" y: "<< std::setprecision(7)<< st.y<< std::fixed
      <<" lat: "<< st.lat
      <<" lon: "<< st.lon
      <<" z: "<< st.z
      <<" speed: "<< st.speed
      <<" yaw: "<< st.yaw/M_PI*180.
      <<" pitch: "<< st.pitch/M_PI*180.
      <<" ax: "<< st.ax<<" ay: "<< st.ay<<" az: "<< st.az<< std::endl;
  std::cout<<"after:"
      <<" t: "<< st1.t
      <<" x: "<< st1.x
      <<" y: "<< std::setprecision(7)<< st1.y<< std::fixed
      <<" lat: "<< st1.lat
      <<" lon: "<< st1.lon
      <<" z: "<< st1.z
      <<" speed: "<< st1.speed
      <<" yaw: "<< st1.yaw/M_PI*180.
      <<" pitch: "<< st1.pitch/M_PI*180.
      <<" ax: "<< st1.ax<<" ay: "<< st1.ay<<" az: "<< st1.az<< std::endl;
}
