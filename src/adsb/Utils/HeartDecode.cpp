#include "HeartDecode.h"
#include <iostream>

namespace Utils{
  
   int HeartDecode(unsigned char *buf, int len, UserStructs::HeartMsg &msg)
   {
     //first make sure the length is ok for heart beat
     int bytes_len= 13;
     if(len!=bytes_len) return 1;
     //first detect 0X7E (dec 126) the flag
     int idx,id0;
     for(idx=0;idx!= len;++idx)
       if( (int)buf[idx]==126 ) break;
     //not found
     if(idx== len) return 2;
     //otherwise check if there is enough space for a whole heartbeat bytes sequence, the lenght of a ADS-B sequence is 36
     if( len- idx <bytes_len) return 3;
     //start from second bytes, checking message ID
     idx+= 1;
     if( (int)buf[idx]!= 0) return 4;
     //then start decoding
     //the second bytes st contains the traffic alert status and address type
     id0= idx;
     idx= id0+ 1;
     //the first byte, bit 7 is if GPS Position valid
     uint8_t B1= buf[idx];
     msg.GPSValid= (B1>>7) & 0x01;
     //byte 2
     idx= id0+2;
     uint8_t B2= buf[idx];
     msg.UTC_OK= B2 & 0x01;
     uint32_t timestamp= (B2>>7) & 0x01;
     //std::cout<<"stamp1: "<< timestamp<< std::endl;
     //byte 3 and 4
     idx= id0+3;
     //LS byte first
     timestamp = (timestamp<<8) + buf[idx+1];
     //std::cout<<"stamp2: "<< timestamp<<" buf2: "<< (int)buf[idx+1]<< std::endl;
     timestamp = (timestamp<<8) + buf[idx];
     //std::cout<<"stamp3: "<< timestamp<<" buf1: "<< (int)buf[idx]<< std::endl;
     msg.TimeStamp= timestamp;
     //std::cout<<"TimeStamp: "<< msg.TimeStamp<< std::endl;
     return 0;
   }//HeartDecode ends
};
