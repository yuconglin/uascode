#include "MavlinkReceiver.hpp"
#include "yc_common/mavlink.h"
#include "Utils/PortOperates.h"

#include "stdio.h"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>
//ros msg
//for goal wa/ypoint
#include "uascode/PosSetPoint.h"
//for position and attitude
#include "uascode/GlobalPos.h" //next to create the ros msgs
#include "uascode/PlaneAttitude.h"

namespace UasCode{
  MavlinkReceiver::MavlinkReceiver():baudrate(115200)
  {
     char* _uartname= "/dev/ttyACM0";
     uart_name= new char[strlen(_uartname)+1];
     strcpy(uart_name,_uartname);
     if_hil= false;
     //publisher initialization
     pub_pos =nh.advertise<uascode::GlobalPos>("global_position", 1);
     pub_att= nh.advertise<uascode::PlaneAttitude>("plane_att",1);
     pub_goal= nh.advertise<uascode::PosSetPoint>("position_setpoint",1);
  }

  MavlinkReceiver::MavlinkReceiver(const char* _uartname,int _baudrate)
    :baudrate(_baudrate)
  {
    uart_name= new char[strlen(_uartname)+1];
    strcpy(uart_name,_uartname);
    if_hil= false;
    //publisher initialization
    pub_pos =nh.advertise<uascode::GlobalPos>("global_position", 1);
    pub_att= nh.advertise<uascode::PlaneAttitude>("plane_att",1);
    pub_goal= nh.advertise<uascode::PosSetPoint>("position_setpoint",1);
  }

  MavlinkReceiver::~MavlinkReceiver(){
    if(uart_name!=NULL) delete uart_name;
  }

  void MavlinkReceiver::SetUartname(const char* _uartname)
  {
    uart_name= new char[strlen(_uartname)+1];
    strcpy(uart_name,_uartname);
  }

  int MavlinkReceiver::OpenPort()
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

  bool MavlinkReceiver::SetupPort()
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
       printf("\nport READY.\n");
       return setup;
  }//SetupPort()

  void MavlinkReceiver::ClosePort()
  {
       Utils::close_port(fd);
  }//ClosePort() ends

  int MavlinkReceiver::ReceiveMsg()
  {
  	mavlink_status_t lastStatus;
	lastStatus.packet_rx_drop_count = 0;
        // Blocking wait for new data
        while(ros::ok() )
	{
	    //std::cout<<"ros while"<< std::endl;  
	    uint8_t cp;
	    mavlink_message_t message;
	    mavlink_status_t status;
	    uint8_t msgReceived = false;

	    if (read(fd, &cp, 1) > 0)
	    {
		    // Check if a message could be decoded, return the message in case yes
		    msgReceived = mavlink_parse_char(MAVLINK_COMM_1, cp, &message, &status);
					    
		    lastStatus = status;
	    }
	    // If a message could be decoded, handle it
	    if(msgReceived)
	    {
            handle_message(&message);
	    }
	    ros::spinOnce();
	}
	return 0;
   
  }//ReceiveMsg ends

  //private functions to dealwith mavlink messages
  void MavlinkReceiver::handle_message(mavlink_message_t *msg){
    switch(msg->msgid){
      
      case MAVLINK_MSG_ID_ATTITUDE:
	handle_message_attitude(msg);
	break;
      
      case MAVLINK_MSG_ID_GPS_RAW_INT:
	handle_message_gps_raw(msg);
	break;
      
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
	handle_message_global_pos(msg);
	break;

      case MAVLINK_MSG_ID_YC_HIL_ATTITUDE:
	handle_message_yc_hil_attitude(msg);
	break;

      case MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_INT:        
	handle_message_global_pos_setpoint(msg);
        break;
    }
  }

  void MavlinkReceiver::handle_message_attitude(mavlink_message_t *msg)
  {
     if(if_hil) return;
     mavlink_attitude_t att;
     mavlink_msg_attitude_decode(msg,&att);
     printf("attitude received:roll:%.4f,pitch:%.4f,yaw:%.4f\n",att.roll*180./M_PI,att.pitch*180./M_PI,att.yaw*180./M_PI);
     //send to ros
     uascode::PlaneAttitude pl_att;
     pl_att.roll= att.roll;
     pl_att.pitch= att.pitch;
     pl_att.yaw= att.yaw;
     pub_att.publish(pl_att);
  }//ends

  void MavlinkReceiver::handle_message_gps_raw(mavlink_message_t *msg)
  {
    //if(!if_hil) if_hil= true;
    mavlink_gps_raw_int_t gps_raw;
    mavlink_msg_gps_raw_int_decode(msg,&gps_raw);
    printf("gps_raw received:lat:%.4f,lon:%.4f,alt:%.1f,cog:%.2f,vel:%.4f\n",gps_raw.lat/(float)1E7,gps_raw.lon/(float)1e7,gps_raw.alt/(float)1e3,gps_raw.cog/(float)1e2,gps_raw.vel/(float)1e2);
    //send to ros
    uascode::GlobalPos global_pos;
    global_pos.lat= gps_raw.lat/(float)1e7;
    global_pos.lon= gps_raw.lon/(float)1e7;
    global_pos.alt= gps_raw.alt/(float)1e3;
    global_pos.cog= gps_raw.cog/(float)1e2*M_PI/180.;
    global_pos.speed= gps_raw.vel/(float)1e2;
    pub_pos.publish(global_pos);
  }

  void MavlinkReceiver::handle_message_global_pos(mavlink_message_t *msg)
  {
    //yet to be defined
    //printf("global_position received\n");
    if(if_hil) return;
    mavlink_global_position_int_t pos;
    mavlink_msg_global_position_int_decode(msg,&pos);
    
    printf("global pos received:lat:%.4f,lon:%.4f,alt:%.1f,hdg:%.2f\n",pos.lat/(float)1E7,pos.lon/(float)1e7,pos.alt/(float)1e3,pos.hdg/(float)1e2);

    //send to ros
    uascode::GlobalPos global_pos;
    global_pos.lat= pos.lat/(float)1e7;
    global_pos.lon= pos.lon/(float)1e7;
    global_pos.alt= pos.alt/(float)1e3;
    global_pos.cog= pos.hdg/(float)1e2*M_PI/180.;
    global_pos.speed= sqrt(pos.vx*pos.vx+pos.vy*pos.vy+pos.vz*pos.vz)/(float)1e2;
    pub_pos.publish(global_pos);
  }

  void MavlinkReceiver::handle_message_yc_hil_attitude(mavlink_message_t *msg)
  {
    if(!if_hil) if_hil= true; 
    mavlink_yc_hil_attitude_t hil_att;
    mavlink_msg_yc_hil_attitude_decode(msg, &hil_att);
    
    printf("yc_hil_att received:roll:%.4f,pitch:%.4f,yaw:%.4f\n",hil_att.roll*180./M_PI,hil_att.pitch*180./M_PI,hil_att.yaw*180./M_PI);

    //send to ros
    uascode::PlaneAttitude pl_att;
    pl_att.roll= hil_att.roll;
    pl_att.pitch= hil_att.pitch;
    pl_att.yaw= hil_att.yaw;
    pub_att.publish(pl_att);
  }

  void MavlinkReceiver::handle_message_global_pos_setpoint(mavlink_message_t *msg)
  {
    mavlink_global_position_setpoint_int_t pos_set;
    mavlink_msg_global_position_setpoint_int_decode(msg,&pos_set);
    //print 
    printf("pos setpoint received:lat:%.4f,lon:%.4f,alt:%.1f,cog:%.2f,vel:%.4f\n",pos_set.latitude/(float)1E7,pos_set.longitude/(float)1e7,pos_set.altitude/(float)1e3);

    //send to ros
    uascode::PosSetPoint pos_msg;
    pos_msg.lat= pos_set.latitude/float(1e7);
    pos_msg.lon= pos_set.longitude/float(1e7);
    pos_msg.alt= pos_set.altitude/float(1e3);
    pub_goal.publish(pos_msg);
  }

}//namespace ends
