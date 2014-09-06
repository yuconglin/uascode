#include "MavlinkSender.hpp"
#include "yc_common/mavlink.h"
#include "common/Utils/YcLogger.h"
#include "Planner/UserStructs/obstacle3D.h"
#include "common/Utils/UTMtransform.h"
#include "common/UserStructs/constants.h"

#include <cstring>

#include <arpa/inet.h>

namespace{
  Utils::LoggerPtr s_logger(Utils::getLogger("uascode.MavlinkSender.YcLogger"));
}

namespace UasCode{

MavlinkSender::MavlinkSender()
{

}

MavlinkSender::~MavlinkSender()
{

}

int MavlinkSender::initialize()
{
    //http://beej.us/guide/bgnet/output/html/multipage/clientserver.html#simpleserver
    UASLOG(s_logger,LL_DEBUG,"initialize starts");

    struct addrinfo hints, *servinfo;
	int rv;

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_DGRAM;

    if ((rv = getaddrinfo("127.0.0.1", UasCode::MAVLINK_OUT_PORT.c_str(), &hints, &servinfo)) != 0) {
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
    UASLOG(s_logger,LL_DEBUG,"initialize ends");
}//initialize ends

void MavlinkSender::SendPosSP(double lat,double lon,double alt)
{
 //lat,lon,alt to mavlink msg
   UASLOG(s_logger,LL_DEBUG,"send lat: " << lat
          << " lon: "<< lon
          << " alt: "<< alt
          );

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
   //write to tcp port
   int numbytes;
   numbytes = sendto(sockfd, buf, len, 0,p->ai_addr, p->ai_addrlen);
}//SetPosSP ends

void MavlinkSender::SendPosSPflag(double lat, double lon, double alt, int index, int inter_exist)
{
    UASLOG(s_logger,LL_DEBUG,"send lat: " << lat
           << " lon: "<< lon
           << " alt: "<< alt
           << " flag: "<< inter_exist);

    mavlink_global_position_setpoint_flag_int_t point_flag_t;
    point_flag_t.latitude= lat*1E7;
    point_flag_t.longitude= lon*1E7;
    point_flag_t.altitude= alt*1E3;
    point_flag_t.index = index;
    point_flag_t.inter_exist= inter_exist;
    point_flag_t.coordinate_frame= MAV_FRAME_GLOBAL;
    //sending
    mavlink_message_t message;
    char buf[128];
    mavlink_msg_global_position_setpoint_flag_int_encode(255,0,&message,&point_flag_t);
    unsigned len= mavlink_msg_to_send_buffer((uint8_t*)buf,&message);
    //write to tcp port
    int numbytes;
    numbytes = sendto(sockfd, buf, len, 0,p->ai_addr, p->ai_addrlen);
}//SendPosSPflag

void MavlinkSender::SendIfColli(bool if_colli)
{
    UASLOG(s_logger,LL_DEBUG,"send if_colli: "<< if_colli);

    mavlink_if_collision_t mav_ifcolli;
    mav_ifcolli.if_collision= (int)if_colli;
    //sending
    mavlink_message_t message;
    char buf[128];
    mavlink_msg_if_collision_encode(255,0,&message,&mav_ifcolli);
    unsigned len= mavlink_msg_to_send_buffer((uint8_t*)buf,&message);

    //write to tcp port
    int numbytes;

    numbytes = sendto(sockfd,buf,len,0,p->ai_addr, p->ai_addrlen);
}// SendIfColli

void MavlinkSender::SendMultiObs(std::vector<UserStructs::obstacle3D> obss)
{
    mavlink_multi_adsb_t adsb_obss;
    int size= obss.size();
    adsb_obss.number= size>5 ? 5:size;

    UASLOG(s_logger,LL_DEBUG,"obss to send: "<< (int)adsb_obss.number);

    for(int i=0;i!= adsb_obss.number;++i)
    {
       adsb_obss.addrs[i]= obss[i].address;
       UASLOG(s_logger,LL_DEBUG,"obs address:"
              << (int)adsb_obss.addrs[i] );

       double lat,lon;
       Utils::FromUTM(obss[i].x1,obss[i].x2,lon,lat);

       adsb_obss.lats[i]= lat;
       adsb_obss.lons[i]= lon;
       adsb_obss.alts[i]= obss[i].x3;

       UASLOG(s_logger,LL_DEBUG,"obs lat,lon,alt:"<< adsb_obss.lats[i]<< " "
             << adsb_obss.lons[i]<< " "
             << adsb_obss.alts[i]<< " "
              );

       double hd= M_PI/2- obss[i].head_xy;
       if(hd<0)
           hd+= 2*M_PI;
       hd= hd / UasCode::DEG2RAD;
       adsb_obss.yaws[i]= hd;

       UASLOG(s_logger,LL_DEBUG,"obss yaw:"
              << adsb_obss.yaws[i]);
    }//for ends

    //set not set ones to zero
    for(int i= adsb_obss.number;i!=5;++i)
    {
       adsb_obss.addrs[i]=0;
       adsb_obss.lats[i]=0.;
       adsb_obss.lons[i]=0.;
       adsb_obss.alts[i]=0.;
       adsb_obss.yaws[i]=0.;
    }

    //sending
    mavlink_message_t message;
    char buf[128];
    mavlink_msg_multi_adsb_encode(200,0,&message,&adsb_obss);
    unsigned len=mavlink_msg_to_send_buffer((uint8_t*)buf,&message);

    //write to tcp port
    int numbytes;

    numbytes = sendto(sockfd,buf,len,0,p->ai_addr, p->ai_addrlen);
}//SendMultiObs

void MavlinkSender::SendWpNum(int wp_num)
{
    UASLOG(s_logger,LL_DEBUG,"send wp_number: "<< wp_num);
    mavlink_wp_number_t wpnum_t;
    wpnum_t.number= wp_num;
    //sending
    mavlink_message_t message;
    char buf[128];
    mavlink_msg_wp_number_encode(255,0,&message,&wpnum_t);
    unsigned len= mavlink_msg_to_send_buffer((uint8_t*)buf,&message);
    //write to tcp port
    int numbytes;
    numbytes = sendto(sockfd,buf,len,0,p->ai_addr, p->ai_addrlen);
}//SendWpNum ends

void MavlinkSender::SendMultiObs3(std::vector<UserStructs::obstacle3D> obss)
{
    mavlink_multi_adsb3_t adsb_obss;
    int size= obss.size();
    adsb_obss.number= size>3 ? 3:size;

    UASLOG(s_logger,LL_DEBUG,"obss to send: "<< (int)adsb_obss.number);

    for(int i=0;i!= adsb_obss.number;++i)
    {
       adsb_obss.addrs[i]= obss[i].address;
       UASLOG(s_logger,LL_DEBUG,"obs address:"
              << (int)adsb_obss.addrs[i] );

       double lat,lon;
       Utils::FromUTM(obss[i].x1,obss[i].x2,lon,lat);

       adsb_obss.lats[i]= lat;
       adsb_obss.lons[i]= lon;

       UASLOG(s_logger,LL_DEBUG,"obs lat,lon,alt:"<< adsb_obss.lats[i]<< " "
             << adsb_obss.lons[i]);

       double hd= M_PI/2- obss[i].head_xy;
       if(hd<0)
           hd+= 2*M_PI;
       hd= hd / UasCode::DEG2RAD;
       adsb_obss.yaws[i]= hd;

       UASLOG(s_logger,LL_DEBUG,"obss yaw:"
              << adsb_obss.yaws[i]);
    }//for ends

    //set not set ones to zero
    for(int i= adsb_obss.number;i!=3;++i)
    {
       adsb_obss.addrs[i]=0;
       adsb_obss.lats[i]=0.;
       adsb_obss.lons[i]=0.;
       adsb_obss.yaws[i]=0.;
    }

    //sending
    mavlink_message_t message;
    char buf[128];
    mavlink_msg_multi_adsb3_encode(200,0,&message,&adsb_obss);
    unsigned len=mavlink_msg_to_send_buffer((uint8_t*)buf,&message);

    //write to tcp port
    int numbytes;

    numbytes = sendto(sockfd,buf,len,0,p->ai_addr, p->ai_addrlen);
}//MultiObssSend3

void MavlinkSender::SendColliPt(double lat_c, double lon_c, double alt_c)
{
    UASLOG(s_logger,LL_DEBUG,"colli point:"<< lat_c <<" "<< lon_c<<" "<< alt_c);
    mavlink_colli_point_t colli_t;
    colli_t.lat= lat_c;
    colli_t.lon= lon_c;
    colli_t.alt= alt_c;
    //sending
    mavlink_message_t message;
    char buf[128];
    mavlink_msg_colli_point_encode(255,0,&message,&colli_t);
    unsigned len= mavlink_msg_to_send_buffer((uint8_t*)buf,&message);
    //write to tcp port
    int numbytes;
    numbytes = sendto(sockfd,buf,len,0,p->ai_addr, p->ai_addrlen);
}

}
