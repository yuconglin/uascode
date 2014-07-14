#include "MavlinkSender.hpp"
#include "yc_common/mavlink.h"
#include <cstring>

#include <arpa/inet.h>
#define SERVERPORT "5760"	

namespace UasCode{
//http://beej.us/guide/bgnet/output/html/multipage/clientserver.html#simpleserver
int MavlinkSender::initialize()
{
  /*ip4addr.sin_family = AF_INET;
  ip4addr.sin_port = htons(14550);
  inet_pton(AF_INET, "127.0.0.1", &ip4addr.sin_addr);
  sockfd = socket(AF_INET, SOCK_DGRAM, 0);
   if( bind(sockfd, (struct sockaddr*)&ip4addr, sizeof(ip4addr) ) <0)
  {
    perror("bind failed");
	      return 0;
  }
  std::cout<<"initialized"<< std::endl;*/
	struct addrinfo hints, *servinfo;
	int rv;
	int numbytes;
	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;

	if ((rv = getaddrinfo("127.0.0.1", "19550", &hints, &servinfo)) != 0) {
		fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
		return 1;
	}

	// loop through all the results and make a socket
	for(p = servinfo; p != NULL; p = p->ai_next) {
	     if ((sockfd = socket(p->ai_family, p->ai_socktype,p->ai_protocol)) == -1) {
    perror("talker: socket");
	continue;
 }

		break;
	}

	if (p == NULL) {
		fprintf(stderr, "talker: failed to bind socket\n");
		return 2;
	}
	 
}//initialize ends

void MavlinkSender::SendPosSP(double lat,double lon,double alt)
{
 //lat,lon,alt to mavlink msg
   mavlink_global_position_setpoint_int_t setpoint;
   setpoint.latitude= lat*1E7;
   setpoint.longitude= lon*1E7;
   setpoint.altitude= alt*1000;
   setpoint.coordinate_frame= MAV_FRAME_GLOBAL;
   //sending
   mavlink_message_t message;
   char buf[128];
   mavlink_msg_global_position_setpoint_int_encode(255,0,&message,&setpoint);
   unsigned len= mavlink_msg_to_send_buffer((uint8_t*)buf,&message);
//std::cout<<"aaa"<< std::endl;

   //write to tcp port
   int numbytes;
   /* 
   if ( ( numbytes = sendto(sockfd, buf, len, 0, (struct sockaddr *)&ip4addr, sizeof(ip4addr)) ) == -1) {
//std::cout<<"numbytes:"<< numbytes<<std::endl;
	        perror("talker: sendto");
        exit(1);
    }*/
   numbytes = sendto(sockfd, buf, len, 0,p->ai_addr, p->ai_addrlen);
   //numbytes= write(sockfd,buf,len);
   std::cout<<"numbytes:"<< numbytes<<std::endl;

   }//SetPosSP ends

};
