#pragma once
#include "yc_common/mavlink.h"
//for socket
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#define BUFFER 1024

namespace UasCode{

class MavlinkTCP{
 public:
  //******constructor	
  int SetUp();
  //********for receiving msg
  bool ReceiveMsg();
  //********for inspecting message
  inline mavlink_message_t GetMessage(){return this->message;}
 private:
  int fd;//file descriptor
  struct sockaddr_in remaddr; /*remote address*/
  //*******mavlink message
  mavlink_message_t message;
  //*******functions to handle messages
  void handle_message(mavlink_message_t *msg);
  void handle_message_attitude(mavlink_message_t *msg); 
};//class ends

}//namespace ends
