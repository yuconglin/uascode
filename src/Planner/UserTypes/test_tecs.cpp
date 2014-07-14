#include "TECsSim.hpp"
#include "common/UserStructs/constants.h"
//std
#include <sstream>
#include <string>
#include <fstream>
#include <iostream>
#include <map>
//other
#include "armadillo"

using namespace UasCode;

std::map<std::string,float*> name_map;
void read_param(char* filename);

int main(int argc,char** argv)
{
  UasCode::TECsSim tecs;
  tecs._load_param_from_file("/home/yucong/yucong_codes_git/parameters.txt");
  //SET dt
  //tecs.set_dt(1.0);
  //test update_50hz
  float baro_alt= 1000;
  arma::vec::fixed<3> accel;
  accel << 0.1 << 0.02 << 0.03;
  tecs.update_50hz(baro_alt,accel);
  //test update_pitch_throttle
  float eas2tas=1.0;
  float pitch= 0.;
  float speed_trim= 30.;
  float speed= 25;
  bool climbOutDem= false;
  //
  for(int i=0;i<1;++i){
     tecs.update_pitch_throttle(pitch, baro_alt, baro_alt, speed_trim, speed, eas2tas, false);
    //get
    std::cout<<"pitch: "<<tecs.get_pitch_demand()
           <<" throttle: "<<tecs.get_throttle_demand()
	   << std::endl;
  }
}//main ends
