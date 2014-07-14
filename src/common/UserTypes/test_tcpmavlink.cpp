#include "MavlinkTCP.hpp"

using namespace UasCode;
int main(int argc,char** argv)
{
 MavlinkTCP mavlink_tcp;
 mavlink_tcp.SetUp();
 while(1){
   if(mavlink_tcp.ReceiveMsg() ){
      //std::cout<<"msg received"<< std::endl;
      mavlink_message_t msg= mavlink_tcp.GetMessage();
      //std::cout<<"msg id: "<< (int)msg.msgid<<std::endl;
      if(msg.msgid == MAVLINK_MSG_ID_INTER_RECEIVE){
         std::cout<<"ros inter receive" << std::endl;
      }

      if(msg.msgid == MAVLINK_MSG_ID_RAW_IMU){
          std::cout<<"raw imu received" << std::endl;
      }

      if(msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT){
          std::cout<<"global position received" << std::endl;
      }

      if(msg.msgid == MAVLINK_MSG_ID_ATTITUDE){
          std::cout<<"attitude received" << std::endl;
      }

      if(msg.msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {
          std::cout<<"highres_imu received" << std::endl;
      }

      if(msg.msgid == MAVLINK_MSG_ID_SIM_STATE) {
          std::cout<<"sim_state received" << std::endl;
      }

   }//if mavlink_tcp.ReceiveMsg() ends
 
 }
}
