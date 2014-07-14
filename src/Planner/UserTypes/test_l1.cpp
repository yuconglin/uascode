#include "L1ControlSim.hpp"
#include "armadillo"
#include <iostream>
using namespace UasCode;

int main(int argc,char** argv)
{
  L1ControlSim l1_control;
  arma::vec::fixed<2> pt_A;
  arma::vec::fixed<2> pt_B;
  arma::vec::fixed<2> curr_posi;
  arma::vec::fixed<2> gnd_speed;
  //assign
  //pt_A<<33.415828<<-111.920080;
  //pt_B<<33.415622<<-111.960893;
  //curr_posi<<33.416267<< -111.934800;
  pt_A<<33.415828<<-111.920080;
  pt_B<<33.415828<<-111.960893;
  curr_posi<<33.415828<<-111.934800;
  gnd_speed<<-10<< -10;
  //navigate_waypoints
  l1_control.navigate_waypoints(pt_B,pt_A,curr_posi,gnd_speed);
  //see caculated target roll and bearing
  std::cout<<"d_roll: "<<l1_control.nav_roll()/M_PI*180.<<" d_bearing: "<<l1_control.nav_bearing()/M_PI*180.<< std::endl;
  return 0;
}
