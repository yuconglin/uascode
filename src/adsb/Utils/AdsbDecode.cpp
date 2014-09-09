#include "AdsbDecode.h"
#include <math.h>
#include <iostream>

namespace Utils{
  
  int AdsbDecode(unsigned char *buf, int len, UserStructs::AdsbMsg &msg)
  {
   //function to convert a char array into AdsbMsg data structure
   //first make sure the length is ok for ADS-B bytes sequence
   int bytes_len= 36;
   if(len!=bytes_len) return 1;
   //first detect 0X7E (dec 126) the flag
   int idx,id0;
   for(idx=0;idx!= len;++idx)
    if( (int)buf[idx]==126 ) break;
   //not found
   if(idx== len) return 2;
   //otherwise check if there is enough space for a whole ADS-B bytes sequence, the lenght of a ADS-B sequence is 36
   if( len- idx <bytes_len) return 3;
   //start from second bytes, checking message ID
   idx+= 1;
   //if( (int)buf[idx]!= 20) return 4;
   //then start decoding
   //the second bytes st contains the traffic alert status and address type
   id0= idx;
   //see traffic source
   //next 1 byte contains px
   idx= id0+27;
   uint8_t PX= buf[idx];
   msg.p= (PX>>4) & 0x0F;
   msg.x= PX & 0x0F;
   //if(msg.x== 3) return 1090; 
   idx= id0+ 1;
   uint8_t ST= buf[idx];
   msg.alert= (ST>>4)&0x0F;
   msg.AddressType= ST&0x0F;
   //bytes aa aa aa for participant address
   idx= id0+2;
   uint32_t addr =0;
   addr= buf[idx];
   //std::cout<<"addr1: "<< addr<< std::endl;
   addr= (addr << 8) + buf[idx+1];
   //std::cout<<"addr2: "<< addr<< std::endl;
   addr= (addr << 8) + buf[idx+2];
   //std::cout<<"addr3: "<< addr<< std::endl;
   msg.address= addr;
   //bytes ll ll ll for latitude
   idx= id0+5;
   int32_t latl =0;
   latl= buf[idx];
   latl= (latl<<8)+buf[idx+1];
   latl= (latl<<8)+buf[idx+2];
   //if negative
   //if( (buf[idx]>>7)&0x01==1 ) latl= ~latl+1;  
   msg.latitude= 180./pow(2,23)* latl;
   //bytes nn nn nn for longitude
   idx= id0+8;
   int32_t lonl =0;
   lonl= buf[idx];
   lonl= (lonl<<8)+buf[idx+1];
   lonl= (lonl<<8)+buf[idx+2];
   //if negative
   //if( (buf[idx]>>7)&0x01==1 ) lonl= ~lonl+1; 
   msg.longitude= 180./pow(2,23)* lonl -360.0;
   //next two byte contains ddd and m
   idx= id0+11;
   uint8_t DD= buf[idx];
   uint8_t DM= buf[idx+1];
   msg.Mi= DM&0x0F;
   uint16_t ddd =0;
   ddd= DD;
   uint16_t d= (DM>>4) & 0X0F;
   uint16_t d1= ddd<<4;
   //std::cout<<"dd: "<<ddd<<" d:"<<d<<" d1: "<<d1<< std::endl;
   ddd= (ddd<<4) + ((DM>>4) & 0x0F);
   //std::cout<<"after add: ddd: "<< ddd << std::endl;
   msg.altitude = ddd*25-1000;
   //next byte contain i a
   idx= id0+13;
   uint8_t IA = buf[idx];
   msg.NIC=(IA>>4)&0x0F;
   msg.NACp=IA&0x0F;
   //next 3 bytes hh hv vv
   idx= id0+14;
   uint8_t HH= buf[idx];
   uint8_t HV= buf[idx+1];
   uint8_t VV= buf[idx+2];
   uint16_t hhh=0;
   hhh= HH;
   hhh= (hhh<<4)+((HV>>4)&0x0F);
   int16_t vvv=0;
   vvv= HV & 0x0F;
   vvv= (vvv<<8)+ VV;
   
   if( (HV>>3)&0x01==1 )
       vvv= -1*(~vvv+1);
   msg.v= hhh;
   msg.vv= vvv*64;
   //next byte is tt, the heading
   idx= id0+17;
   msg.hd =buf[idx]*360./256;
   //next byte is ee, emitter category
   idx= id0+18;
   msg.EC =buf[idx];
   //next 8 bytes is call sign
   idx= id0+19;
   for(int i=0;i!=8;++i) 
     msg.CallSign[i]= buf[idx+i];
      //ends and return
   return 0;

  }//AdsbDecode ends

}
