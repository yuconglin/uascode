#include "SageDecoder.hpp"
//#include "Utils/AdsbDecode.h"
//#include "Utils/HeartDecode.h"
#include "Utils/Decode.h"
#include <iostream>

namespace UasCode{

int SageDecoder::Decode(unsigned char *buf, int len)
{
   int idx;
   for(idx=0;idx!= len;++idx)
    if( (int)buf[idx]==126 ) break;
   //see message id
   int msg_re= -10;
   switch( (int)buf[idx+1] ){
     case 60:
       std::cout<<"status"<< std::endl;
       msg_re= Utils::StatusDecode(buf, len, sage_status); 
       break;
     case 62:
       break;
     case 10:
       msg_re= Utils::AdsbDecode(buf, len, ownership.bodymsg);
       break;
     case 20:
       msg_re= Utils::AdsbDecode(buf, len, sage_adsb);
       break;
     case 0:
       msg_re= Utils::HeartDecode(buf, len, heartbeat); 
       break;
     default:
       break;
   }//switch ends
   if(msg_re==0) return (int)buf[idx+1];
   return msg_re;
}//Decode ends

}//namespace UasCode ends
