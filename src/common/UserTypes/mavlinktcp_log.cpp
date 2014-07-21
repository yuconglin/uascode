#include "MavlinkTCP.hpp"
#include "common/Utils/GetTimeNow.h"
#include "common/Utils/GetTimeUTC.h"
#include "common/Utils/UTMtransform.h"
#include <fstream>
#include <cmath>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace UasCode;

int main(int argc, char** argv)
{
 MavlinkTCP mavlink_tcp;
 mavlink_tcp.SetUp();
 bool if_log= false, if_log_pre=false;
 std::fstream fs_state;
 fs_state.open("sitl_state1.txt",std::ofstream::out | std::ofstream::in | std::ofstream::trunc);

 bool if_accel= false, if_posi= false, if_atti= false;
 double ax,ay,az;
 double lat,lon,hgt,speed,hd;
 double roll,pitch,yaw;
 double dvz;
 double vx_pre,vy_pre,vz_pre;
 double t_pre=0,dt=0.3;

 //while loop
 while(1)
 {
   if(mavlink_tcp.ReceiveMsg() )
   {
      mavlink_message_t msg= mavlink_tcp.GetMessage();
      if(msg.msgid == MAVLINK_MSG_ID_INTER_RECEIVE)
      {
         mavlink_inter_receive_t inter_rec;
	 mavlink_msg_inter_receive_decode(&msg, &inter_rec);
	 if(inter_rec.receive==1) 
	   if_log= true;
	 else
           if_log= false;
	 if(!if_log && if_log_pre) break;
      }
      if(if_log){
	          //decode acceleration
	 if(msg.msgid== MAVLINK_MSG_ID_RAW_IMU){
              mavlink_raw_imu_t imu_t;
              mavlink_msg_raw_imu_decode(&msg,&imu_t);
	      ax= imu_t.xacc;
	      ay= imu_t.yacc;
	      az= imu_t.zacc;
	      if_accel= true;
	 }
	 //decode position and speed
	 if(msg.msgid== MAVLINK_MSG_ID_GLOBAL_POSITION_INT){
	      mavlink_global_position_int_t pos_t;
	      mavlink_msg_global_position_int_decode(&msg,&pos_t);
	      lat= pos_t.lat/1E7;
	      lon= pos_t.lon/1E7;
	      hgt= pos_t.alt/1E3;
	      double vx,vy,vz;
	      vx= pos_t.vx/100;
	      vy= pos_t.vy/100;
	      vz= pos_t.vz/100;
              speed= std::sqrt(vx*vx+vy*vy+vz*vz);
              dvz= (vz_pre-vz)/dt;
	      hd= pos_t.hdg/100;

          vx_pre= vx;
          vy_pre= vy;
          vz_pre= vz;

	      if_posi= true;
	 }
	 //decode RPY
	 if(msg.msgid== MAVLINK_MSG_ID_ATTITUDE){
              mavlink_attitude_t att_t;
	      mavlink_msg_attitude_decode(&msg,&att_t);
              roll= att_t.roll;
	      pitch= att_t.pitch;
	      yaw= att_t.yaw;
	      if_atti= true;
	 }
	 std::cout<<"if_accel: "<< if_accel<<" "
		  <<"if_posi: "<< if_posi<<" "
		  <<"if_atti: "<< if_atti<< std::endl;

	 if(if_accel && if_posi && if_atti){
           double t_now= Utils::GetTimeNow();
	   //log into fs_state
           double x, y;
           Utils::ToUTM( lon, lat, x, y );
           fs_state << std::setprecision(5)<< std::fixed 
		   << t_now <<" "
		   << lon <<" "
		   << lat <<" "
		   << hgt <<" "
		   << speed << " "
           << x << " "
           << y << " "
		   << hd << " "
		   << roll << " "
		   << pitch << " "
		   << yaw << " "
           << ax <<" "<< ay <<" "<< az << " "
           << dvz << " "
		   << std::endl;
	   if_accel= false;
	   if_posi= false;
	   if_atti= false;
       dt= t_now- t_pre;
       t_pre= t_now;
	 }
      }//if_log ends
      if_log_pre= if_log;
   }
 }//while ends

}
