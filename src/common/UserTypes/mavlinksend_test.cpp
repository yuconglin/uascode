#include "MavlinkSender.hpp"
#include "common/Utils/YcLogger.h"
#include "MavlinkTCP.hpp"
#include "Utils/GetTimeNow.h"
#include <unistd.h>

int main(int argc,char** argv)
{
  Utils::LogConfigurator myconfigurator("log4cxx_MavSendNode.properties", "log for MavSendNode");
  UasCode::MavlinkSender sender;
  sender.initialize();
  UasCode::MavlinkTCP mavlink_tcp;
  mavlink_tcp.SetUp();
  
  while(1) {
   //sender.SendPosSP(-35.3694553,149.1483307,30);
   sender.SendIfColli(true);
   /*
   if(mavlink_tcp.ReceiveMsg() ){
      mavlink_message_t msg= mavlink_tcp.GetMessage();
      if(msg.msgid == MAVLINK_MSG_ID_INTER_RECEIVE){
          mavlink_inter_receive_t inter_rec;
          mavlink_msg_inter_receive_decode(&msg, &inter_rec);
          if(inter_rec.receive==1) break;
      }
   }//if mavlink_tcp.ReceiveMsg() ends
    */
  }//while ends
  return 0;
}//main ends
