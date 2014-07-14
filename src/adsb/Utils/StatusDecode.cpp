#include "StatusDecode.h"
#include <ctime>
#include <iostream>

namespace Utils{
 int StatusDecode(unsigned char* buf,int len,UserStructs::StatusMsg &msg)
 {
   const clock_t begin_time = clock();
   //first make sure that length is ok for status byte
   int bytes_len= 84, msg_id=60;
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
   if( (int)buf[idx]!= 60) return 4;
   //then start decoding
   //the second bytes st contains the traffic alert status and address type
   id0= idx;
   idx= id0+ 1;
   msg.SeqNum= buf[idx];
   idx= id0+ 4;
   uint8_t B4= buf[idx];
   msg.GPSFix= (B4>>4) & 0x03;
   msg.GPSMode= (B4>>2) & 0X03;
   msg.GPSOK= (B4>>1) & 0x01;
   msg.UTCOK= B4 & 0x01;
   //next byte stallite used
   idx= id0+ 5;
   //uint8_t B5= buf[idx];
   msg.SatUsed= buf[idx];
   msg.SBASSatUsed= buf[idx+1];
   msg.SatInView= buf[idx+2];
   //next 3 bytes GPS HDOP
   idx= id0+ 8;
   int32_t HDOP= buf[idx];
   HDOP= (HDOP<<8) +buf[idx+1];
   HDOP= (HDOP<<8) +buf[idx+2];
   HDOP= (HDOP<<8) +buf[idx+3];
   msg.GPSHDOP= (float)HDOP;
   //next byte 978 count
   idx= id0+ 12;
   uint16_t c978= buf[idx];
   c978= (c978<<8)+ buf[idx+1];
   //next byte 1090 count
   idx= id0+ 16;
   uint16_t c1090= buf[idx];
   c1090= (c1090<<8)+ buf[idx+1];
   //next byte 978 track
   idx= id0+ 18;
   uint16_t t978= buf[idx];
   t978= (t978<<8)+ buf[idx+1];
   //next byte 1090 track
   idx= id0+ 20;
   uint16_t t1090= buf[idx];
   t1090= (t1090<<8)+ buf[idx+1];
   msg.count978= c978;
   msg.count1090= c1090;
   msg.track978= t978;
   msg.track1090= t1090;
   //next temprature and battery
   idx= id0+ 25;
   uint8_t B25= buf[idx];
   msg.AutoShutdown= (B25>>6) & 0x03;
   msg.TempTooHigh= (B25>>3) & 0x01;
   msg.ExPower= (B25>>2) & 0x01;
   msg.ChargeStatus= (B25>>1) & 0x01;
   //next battery percent remain
   idx= id0+ 27;
   msg.BatteryPercent= buf[idx];
   //next byte ground state count
   idx= id0+ 31;
   msg.GSCount= buf[idx];
   //next about NMEA
   idx= id0+ 45;
   uint8_t B46= buf[idx];
   msg.NMEA_GGA= (B46>>5) & 0x01;
   msg.NMEA_GLL= (B46>>4) & 0x01;
   msg.NMEA_GSA= (B46>>3) & 0X01;
   msg.NMEA_GSV= (B46>>2) & 0X01;
   msg.NMEA_RMC= (B46>>1) & 0X01;
   msg.NMEA_VTG= B46 & 0x01;
   //next about max supported targets
   idx= id0+ 46;
   uint16_t B47= buf[idx];
   uint16_t maxsupport = B47;
   maxsupport= (maxsupport<<8)+ buf[idx+1];
   msg.MaxSupport= maxsupport;
   //next about max reported targets
   idx= id0+ 48;
   uint16_t B49= buf[idx];
   uint16_t maxrep= B49;
   maxrep= (maxrep<<8)+ buf[idx+1];
   msg.MaxReport= maxrep;
   //next about LED
   idx= id0+ 56;
   msg.LEDLevel= buf[idx];
   //next about data burst
   idx= id0+ 59;
   msg.DataBurst= buf[idx];
   //next pitch offset
   idx= id0+ 69;
   msg.PitchOffset= buf[idx];
   std::cout << float( clock () - begin_time ) /  CLOCKS_PER_SEC<< std::cout;
   return 0;
 }//StatusDecode ends

};//namespace ends
