#include "MsgSender.hpp"
#include "Utils/PortOperates.h"
#include "yc_common/mavlink.h"

#include "stdio.h"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>
namespace UasCode{
   MsgSender::MsgSender():baudrate(115200)
   {
     char* _uartname= "/dev/ttyACM0";
     uart_name= new char[strlen(_uartname)+1];
     strcpy(uart_name,_uartname);
   }

   MsgSender::MsgSender(char *_uartname,int _baudrate):baudrate(_baudrate)
   {
     uart_name= new char[strlen(_uartname)+1];
     strcpy(uart_name,_uartname);
   }
   
   MsgSender::~MsgSender(){
     if(uart_name!=NULL) 
       delete uart_name;
   }

   void MsgSender::SetUartname(const char *_uartname)
   {
      uart_name= new char[strlen(_uartname)+1];
      strcpy(uart_name,_uartname);
   }

   int MsgSender::OpenPort()
   {
     printf("Trying to connect to %s.. ", uart_name);
     this->fd= Utils::open_port(uart_name);
     if (fd == -1)
	{
		printf("failure, could not open port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		printf("success.\n");
	}

     return (this->fd);
   }//OpenPort ends

   bool MsgSender::SetupPort()
   {
        bool setup = Utils::setup_port(fd, baudrate, 8, 1, false, false);
	if (!setup)
	{
		printf("failure, could not configure port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		printf("success.\n");
	}

	int noErrors = 0;
	
	if(fd < 0)
	{
		exit(noErrors);
	}

	// Run indefinitely while the serial loop handles data
	printf("\nREADY, sending data.\n");
        return setup;
   }//SetupPort()

   void MsgSender::ClosePort()
   {
        Utils::close_port(fd);
   }//ClosePort() ends

   int MsgSender::SendAdsbMsg(const UserStructs::AdsbMsg adsb_m)
   {
        std::cout<<"SendAdsbMsg"<< std::endl;
        mavlink_adsb_short_t adsb_msg;
	adsb_msg.address= adsb_m.address;
	adsb_msg.latitude= adsb_m.latitude;
	adsb_msg.longitude= adsb_m.longitude;
	adsb_msg.altitude= adsb_m.altitude;
	adsb_msg.h_velocity= adsb_m.v;//horizontal velocity in knots
	adsb_msg.v_velocity= adsb_m.vv;//vertical velocity in fpm
        adsb_msg.heading= adsb_m.hd;
	//sending
	mavlink_message_t message;
        char buf[128];
	mavlink_msg_adsb_short_encode(200, 0, &message, &adsb_msg);
        unsigned len= mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
        /* write packet via serial link */ 
        int ret= write(fd, buf, len);
       /* wait until all data has been written */
       tcdrain(fd);
       return ret;
   }

   int MsgSender::SendMultiAdsbs(const std::vector<UserStructs::AdsbQGC> qgc_adsbs)
   {
       std::cout<<"SendMultiAdsbs"<< std::endl;
       //assigning
       mavlink_multi_adsb_t multi_adsb;
       int size= qgc_adsbs.size();
       multi_adsb.number= size > 5? 5:size;
       for(int i=0;i!=multi_adsb.number;++i)
       {
         multi_adsb.addrs[i]= qgc_adsbs[i].address; 
         multi_adsb.lats[i]= qgc_adsbs[i].latitude;
         multi_adsb.lons[i]= qgc_adsbs[i].longitude;
         multi_adsb.alts[i]= qgc_adsbs[i].altitude;
         multi_adsb.yaws[i]= qgc_adsbs[i].hd;
       }//for ends
       
       //sending
       mavlink_message_t message;
       char buf[128];
       mavlink_msg_multi_adsb_encode(200,0,&message,&multi_adsb);
       unsigned len=mavlink_msg_to_send_buffer((uint8_t*)buf,&message);
       //write
       int ret= write(fd,buf,len);
       tcdrain(fd);
       return ret;
   }
}//namespace ends
