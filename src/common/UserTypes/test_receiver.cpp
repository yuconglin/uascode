#include "MavlinkReceiver.hpp"
#include "ros/ros.h"

using namespace UasCode;
int main(int argc,char** argv)
{
  ros::init(argc,argv,"test_receiver");
  //arguments
  char *uart_name = (char*)"/dev/ttyUSB0";
  int baudrate = 57600;
  const char *commandline_usage = "\tusage: %s -d <devicename> -b <baudrate> [-v/--verbose] [--debug]\n\t\tdefault: -d %s -b %i\n";
  
  /* read program arguments */
  for (int i = 1; i < argc; i++) { /* argv[0] is "mavlink" */
	 	      /* UART device ID */
	      if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
		      if (argc > i + 1) {
			      uart_name = argv[i + 1];

		      } else {
			      printf(commandline_usage, argv[0], uart_name, baudrate);
			      return 0;
		      }
	      }

	      /* baud rate */
	      if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
		      if (argc > i + 1) {
			      baudrate = atoi(argv[i + 1]);

		      } else {
			      printf(commandline_usage, argv[0], uart_name, baudrate);
			      return 0;
		      }
	      }
      }

  //MavlinkReceiver receiver("/dev/ttyACM0",115200);
  MavlinkReceiver receiver(uart_name,baudrate);
  receiver.OpenPort();
  receiver.SetupPort();
  receiver.ReceiveMsg();
  return 0;
}
