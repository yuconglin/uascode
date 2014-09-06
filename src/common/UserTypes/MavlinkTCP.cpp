#include "MavlinkTCP.hpp"

//for socket
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
//std
#include "stdio.h"
#include <iostream>
#include "common/UserStructs/constants.h"

//#define PORT 14550

namespace UasCode{
 
// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
  if (sa->sa_family == AF_INET) {
     return &(((struct sockaddr_in*)sa)->sin_addr);
  }
  return &(((struct sockaddr_in6*)sa)->sin6_addr);
}

 int MavlinkTCP::SetUp()
 {
   struct sockaddr_in myaddr;	/* our address */

   /* create a UDP socket */
   if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
	   perror("cannot create socket\n");
	   return 0;
   }
   /* bind the socket to any valid IP address and a specific port */
   memset((char *)&myaddr, 0, sizeof(myaddr));
   myaddr.sin_family = AF_INET;
   myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
   myaddr.sin_port = htons(UasCode::MAVLINK_IN_PORT);

   if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
	   perror("bind failed");
	   return 0;
   }

 }

 bool MavlinkTCP::ReceiveMsg()
 {
     //receiving loop
     socklen_t addrlen = sizeof(remaddr); /*length of addresses */
     unsigned char buf[BUFSIZE]; /* receive buffer */
     char s[INET6_ADDRSTRLEN];
     int recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen); 
     //printf("listener: got packet from %s\n",inet_ntop(remaddr.sin_family,get_in_addr((struct sockaddr *)&remaddr),s, sizeof s));

     //mavlink decode
     mavlink_status_t status;
     uint8_t msgReceived = false;
     if(recvlen>0){
      //printf("recvlen:%d\n",recvlen);
      for(int i=0;i!=recvlen;++i){
        msgReceived = mavlink_parse_char(MAVLINK_COMM_1,buf[i],&message,&status);
	//if(msgReceived) handle_message(&message);
      }
     }//if recvlen > 0 ends
     return msgReceived;
 }//ReceiveMsg ends

 void MavlinkTCP::handle_message(mavlink_message_t *msg){
   switch(msg->msgid){
    case MAVLINK_MSG_ID_ATTITUDE:
     std::cout<<"attitude"<<std::endl;
     break;
   }
 }

};//namespace UasCode ends
