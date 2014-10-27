#include "ObsMonitor.hpp"
#include "Utils/GetTimeUTC.h"
#include "Utils/systemtime.h"
#include "Utils/AdsbMsgToQGC.h"
#include "UserStructs/AdsbMsg.h"
#include "UserStructs/HeartMsg.h"
#include "UserStructs/AdsbQGC.h"
#include "common/Utils/AdsbToObs.h"
#include "Planner/UserStructs/obstacle3D.h"
#include "common/UserStructs/constants.h"

#include <fstream>
#include <iostream>
#include <cstring>
#include <ctime>
//for socket
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
//ros
#include "yucong_rosmsg/MultiObsMsg2.h"
#include "yucong_rosmsg/ObsMsg2.h"
#include "uascode/ObsMsg.h"

namespace UasCode{

   ObsMonitor::ObsMonitor()
   {
      //publisher
      pub_obss=nh.advertise<yucong_rosmsg::MultiObsMsg2>("/mavros/multi_obstacles",100);
   }

   int ObsMonitor::PortSetUp()
   {
       return sender.initialize();
   }

   int ObsMonitor::BytesDecode()
   {  
       struct sockaddr_in myaddr;	/* our address */
       struct sockaddr_in remaddr;	/* remote address */
       socklen_t addrlen = sizeof(remaddr);		/* length of addresses */
       int recvlen;			/* # bytes received */
       int fd;				/* our socket */
       unsigned char buf[BUFSIZE];	/* receive buffer */
       /* create a U<DP socket */
       if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
           perror("cannot create socket\n");
           return 0;
       }
       /* bind the socket to any valid IP address and a specific port */
       memset((char *)&myaddr, 0, sizeof(myaddr));
       myaddr.sin_family = AF_INET;
       myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
       myaddr.sin_port = htons(UasCode::ADSB_IN_PORT);

       if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
           perror("bind failed");
           return 0;
       }

       //file to log bytes
       //get the system time
       std::string str_time;
       Utils::getSystemTime(str_time);

       char file_bytes[256],file_adsb[256],file_afs[256],file_obs[256];
       sprintf(file_bytes, "../data/%s%s.txt",
               str_time.c_str(),"bytes");
       sprintf(file_adsb, "../data/%s%s.txt",
               str_time.c_str(),"adsb");
       sprintf(file_afs, "../data/%s%s.txt",
               str_time.c_str(),"afs");
       sprintf(file_obs, "../data/%s%s.txt",
               str_time.c_str(),"obs");

       //file to log raw bytes
       std::ofstream fs_bytes(file_bytes);
       //file to log decoded data
       std::ofstream fs_adsb(file_adsb);
       //file to log only aircrafts
       std::ofstream fs_afs(file_afs);
       //file to log converted obstacles
       std::ofstream fs_obs(file_obs);

       int count=0;//count for useful byte sequence to decode
       yucong_rosmsg::MultiObsMsg2 obss_msg;
       //decoding bytes
       //while(1)
       ros::Rate r(10);
       while(ros::ok() )
       {
           ros::spinOnce();
           recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
           std::cout<<"recvlen: "<< recvlen<< std::endl;
           int msg_re= decoder.Decode(buf,recvlen);
           if(msg_re!=-10) ++count;
           //bytes log
           if (recvlen > 0) {
               buf[recvlen] = 0;
               fs_bytes<<"received message."<< std::endl;
               for(int i=0;i!=recvlen;++i)
               {
                   fs_bytes<< std::hex<< std::uppercase<< (int)buf[i]<<" ";
                   printf("%X ",buf[i]);
               }
               fs_bytes<< std::endl;
               printf("\n");
           }
           //ownership
           if(msg_re== 10)
           {
               UserStructs::OwnerMsg owner_msg= decoder.GetOwnership();
               UserStructs::AdsbMsg msg= owner_msg.bodymsg;

               fs_adsb << "owner alert:"<<(int)msg.alert<<" "
                       << "address_type:"<<(int)msg.AddressType<<" "
                       << "address:"<<(int)msg.address<<" "
                       << msg.latitude<<" "
                       << msg.longitude<<" "
                       << msg.altitude<<" "
                       << "Mi:"<<(int)msg.Mi<<" "
                       << "NIC:"<<(int)msg.NIC<<" "
                       << "NACp:"<<(int)msg.NACp<<" "
                       << "v:"<<msg.v<<" "
                       << "vv:"<<msg.vv<<" "
                       << "hd:"<<msg.hd<<" "
                       << (int)msg.EC<<" "
                       << msg.CallSign[0]<< msg.CallSign[1]<< msg.CallSign[2]<< msg.CallSign[3]
                       << msg.CallSign[4]<< msg.CallSign[5]<< msg.CallSign[6]<< msg.CallSign[7]
                       << (int)msg.p<<" "
                       << (int)msg.x<< std::endl;
               fs_adsb << std::endl;

               fs_afs << "owner alert:"<<(int)msg.alert<<" "
                      << "address_type:"<<(int)msg.AddressType<<" "
                      << "address:"<<(int)msg.address<<" "
                      << msg.latitude<<" "
                      << msg.longitude<<" "
                      << msg.altitude<<" "
                      << "Mi:"<<(int)msg.Mi<<" "
                      << "NIC:"<<(int)msg.NIC<<" "
                      << "NACp:"<<(int)msg.NACp<<" "
                      << "v:"<<msg.v<<" "
                      << "vv:"<<msg.vv<<" "
                      << "hd:"<<msg.hd<<" "
                      << (int)msg.EC<<" "
                      << msg.CallSign[0]<< msg.CallSign[1]<< msg.CallSign[2]<< msg.CallSign[3]
                      << msg.CallSign[4]<< msg.CallSign[5]<< msg.CallSign[6]<< msg.CallSign[7]
                      << (int)msg.p<<" "
                      << (int)msg.x<< std::endl;

               //converted to obstacles and log
               /*
               //UserStructs::obstacle3D obs;
               yucong_rosmsg::ObsMsg2 msg2;
               //Utils::AdsbToObs(msg,obs);
               Utils::AdsbToObsMsg2(adsb,msg2);
               obss.push_back(obs);
               obss_msg.MultiObs.push_back(ObsToRosMsg(obs) );
               fs_obs<< obs.address<<" "
                     << obs.x1 <<" "<< obs.x2 <<" " << obs.x3<<" "
                     << obs.head_xy*M_PI/180.<<" "<< obs.speed<<" "
                     << obs.v_vert<<" "<< obs.t<<" "
                     << obs.r <<" "<< obs.hr << std::endl;
               */
           }
           //adsb log
           if(msg_re== 20)
           {
               UserStructs::AdsbMsg msg = decoder.GetAdsb();

               //if no repeats find, just insert it
               bool repeat= false;
               for(int i=0;i!= obss2.size();++i)
               {
                   if(msg.address== obss2[i].address){
                       repeat= true;
                       break;
                   }
               }

               if(repeat){
                   /*
                   if(mavlink_send)
                       sender.SendMultiObs3(obss);
                       */
                   //ros publish
                   pub_obss.publish(obss_msg);
                   obss_msg.MultiObs.clear();
               }

               fs_adsb << "alert:"<<(int)msg.alert<<" "
                       << "address_type:"<<(int)msg.AddressType<<" "
                       << "address:"<<(int)msg.address<<" "
                       << msg.latitude<<" "
                       << msg.longitude<<" "
                       << msg.altitude<<" "
                       << "Mi:"<<(int)msg.Mi<<" "
                       << "NIC:"<<(int)msg.NIC<<" "
                       << "NACp:"<<(int)msg.NACp<<" "
                       << "v:"<<msg.v<<" "
                       << "vv:"<<msg.vv<<" "
                       << "hd:"<<msg.hd<<" "
                       << (int)msg.EC<<" "
                       << msg.CallSign[0]<< msg.CallSign[1]<< msg.CallSign[2]<< msg.CallSign[3]
                       << msg.CallSign[4]<< msg.CallSign[5]<< msg.CallSign[6]<< msg.CallSign[7]
                       << (int)msg.p<<" "
                       << (int)msg.x<< std::endl;
               fs_adsb << std::endl;

               fs_afs << "alert:"<<(int)msg.alert<<" "
                      << "address_type:"<<(int)msg.AddressType<<" "
                      << "address:"<<(int)msg.address<<" "
                      << msg.latitude<<" "
                      << msg.longitude<<" "
                      << msg.altitude<<" "
                      << "Mi:"<<(int)msg.Mi<<" "
                      << "NIC:"<<(int)msg.NIC<<" "
                      << "NACp:"<<(int)msg.NACp<<" "
                      << "v:"<<msg.v<<" "
                      << "vv:"<<msg.vv<<" "
                      << "hd:"<<msg.hd<<" "
                      << (int)msg.EC<<" "
                      << msg.CallSign[0]<< msg.CallSign[1]<< msg.CallSign[2]<< msg.CallSign[3]
                      << msg.CallSign[4]<< msg.CallSign[5]<< msg.CallSign[6]<< msg.CallSign[7]
                      << (int)msg.p<<" "
                      << (int)msg.x<< std::endl;

               //converted to obstacles and log
               UserStructs::obstacle3D obs;
               Utils::AdsbToObs(msg,obs);
               yucong_rosmsg::ObsMsg2 msg2;
               Utils::AdsbToObsMsg2(msg,msg2);
               obss_msg.MultiObs.push_back(msg2);

               fs_obs<< msg2.address<<" "<< std::setprecision(8) << std::fixed
                     << obs.x1 <<" "<< obs.x2 <<" " << obs.x3<<" "
                     << obs.head_xy*180./M_PI<<" "<< obs.speed<<" "
                     << obs.v_vert<<" "<< obs.t<<" "
                     << obs.r <<" "<< obs.hr << std::endl;

           }//adsb
           if(msg_re== 0)
           {
               UserStructs::HeartMsg msg= decoder.GetHeart();
               fs_adsb<<"time:"<< msg.TimeStamp<<" "
                     <<"time ref:"<<Utils::GetTimeUTC()<< std::endl;
               fs_adsb << std::endl;
           }//heartbeat
           //device status
           if(msg_re== 60)
           {
               fs_adsb<<"status: "<< std::endl;
               UserStructs::StatusMsg msg= decoder.GetStatus();
               fs_adsb<<"seq: "<<(int)msg.SeqNum<<" "
                     <<"GPSFIX: "<<(int)msg.GPSFix<<" "
                    <<"GPSMode: "<<(int)msg.GPSMode<<" "
                   <<"GPSOK: "<<(int)msg.GPSOK<<" "
                  <<"UTCOK: "<<(int)msg.UTCOK<<" "
                 <<"SatUsed: "<<(int)msg.SatUsed<<" "
                <<"SBASSatUsed: "<<(int)msg.SBASSatUsed<<" "
               <<"SatInView: "<<(int)msg.SatInView<<" "
               <<"GPSHDOP: "<<(float)msg.GPSHDOP<<" "
               <<"count978: "<<(int)msg.count978<<" "
               <<"count1090: "<<(int)msg.count1090<<" "
               <<"track978: "<<(int)msg.track978<<" "
               <<"track1090: "<<(int)msg.track1090<<" "
               <<"AutoShutdown: "<<(int)msg.AutoShutdown<<" "
               <<"TempTooHigh: "<<(int)msg.TempTooHigh<<" "
               <<"Expower: "<<(int)msg.ExPower<<" "
               <<"ChargeStatus: "<<(int)msg.ChargeStatus<<" "
               <<"BatteryPercent: "<<(int)msg.BatteryPercent<<" "
               <<"GSCount: "<<(int)msg.GSCount<<" "
               <<"GGA: "<<(int)msg.NMEA_GGA<<" "
               <<"GLL: "<<(int)msg.NMEA_GLL<<" "
               <<"GSA: "<<(int)msg.NMEA_GSA<<" "
               <<"GSV: "<<(int)msg.NMEA_GSV<<" "
               <<"RMC: "<<(int)msg.NMEA_RMC<<" "
               <<"VTC: "<<(int)msg.NMEA_VTG<<" "
               <<"MaxSupport: "<<(int)msg.MaxSupport<<" "
               <<"MaxReport: "<<(int)msg.MaxReport<<" "
               <<"LEDLevel: "<<(int)msg.LEDLevel<<" "
               <<"DataBurst: "<<(int)msg.DataBurst<<" "
               <<"PitchOffset: "<<(int)msg.PitchOffset<< std::endl;
           }//status ends
           //ros::spinOnce();
       }//while ends
       return count;
   }//BytesDecode ends

   uascode::ObsMsg ObsToRosMsg(const UserStructs::obstacle3D& obs)
   {
       uascode::ObsMsg obs_msg;
       obs_msg.address= obs.address;
       obs_msg.x1= obs.x1;
       obs_msg.x2= obs.x2;
       obs_msg.x3= obs.x3;
       obs_msg.head_xy= obs.head_xy;
       obs_msg.speed= obs.speed;
       obs_msg.v_vert= obs.v_vert;
       obs_msg.t= obs.t;
       obs_msg.r= obs.r;
       obs_msg.hr= obs.hr;
       return obs_msg;
   }

}//namespace ends

