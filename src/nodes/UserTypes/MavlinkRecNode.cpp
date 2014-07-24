#include "MavlinkRecNode.hpp"
#include "common/UserStructs/constants.h"
#include "ros/ros.h"

namespace UasCode{

MavlinkRecNode::MavlinkRecNode()
{
  //publisher
  pub_posi= nh.advertise<uascode::GlobalPos>("global_position",100);
  pub_att= nh.advertise<uascode::PlaneAttitude>("plane_att",100);
  pub_IfRec= nh.advertise<uascode::IfRecMsg>("interwp_receive",100);
  pub_accel= nh.advertise<uascode::AccelXYZ>("accel_raw_imu",100);
  pub_wp_current= nh.advertise<uascode::WpCurrent>("waypoint_current",100);
  pub_if_mavlink= nh.advertise<uascode::IfMavlinkGood>("if_mavlink",100);
}

MavlinkRecNode::~MavlinkRecNode(){}

void MavlinkRecNode::TcpSetUp()
{
   mavlink_tcp.SetUp();
}

void MavlinkRecNode::working()
{
   //ros::Rate r(10);
   while(ros::ok() )
   {
     ros::spinOnce();

     bool if_MavLink= false;

     if(mavlink_tcp.ReceiveMsg())
     {
       /*we need to add a message
        to indicate if mavlink still
        available */
       if_MavLink= true;

       mavlink_message_t msg= mavlink_tcp.GetMessage();
       //IF INTER WP RECEIVED
       if(msg.msgid == MAVLINK_MSG_ID_INTER_RECEIVE)
       {
         mavlink_inter_receive_t inter_rec;
         mavlink_msg_inter_receive_decode(&msg,&inter_rec);
         if_rec.receive= (bool)inter_rec.receive;
         pub_IfRec.publish(if_rec);
       }
       //IF RAW IMU (ACCEL) RECEIVED
       if(msg.msgid == MAVLINK_MSG_ID_RAW_IMU)
       {
         mavlink_raw_imu_t raw_imu_t;
         mavlink_msg_raw_imu_decode(&msg,&raw_imu_t);
         accel_xyz.ax= (double)raw_imu_t.xacc/1000*CONSTANT_G;
         accel_xyz.ay= (double)raw_imu_t.yacc/1000*CONSTANT_G;
         accel_xyz.az= (double)raw_imu_t.zacc/1000*CONSTANT_G;
         pub_accel.publish(accel_xyz);
       }
       //IF GLOBAL POSITION RECEIVED
       if(msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
       {
         mavlink_global_position_int_t pos_t;
         mavlink_msg_global_position_int_decode(&msg,&pos_t);
         global_pos.lat= pos_t.lat/1E7;
         global_pos.lon= pos_t.lon/1E7;
         global_pos.alt= pos_t.alt/1E3;
         double vx,vy,vz;
         vx= pos_t.vx/100;
         vy= pos_t.vy/100;
         vz= pos_t.vz/100;
         global_pos.speed= std::sqrt(vx*vx+vy*vy+vz*vz);
         global_pos.cog= pos_t.hdg/100;
         pub_posi.publish(global_pos);
       }
       //IF ATTITUDE RECEIVED
       if(msg.msgid == MAVLINK_MSG_ID_ATTITUDE)
       {
         mavlink_attitude_t att_t;
         mavlink_msg_attitude_decode(&msg,&att_t);
         plane_att.roll= att_t.roll;
         plane_att.pitch= att_t.pitch;
         plane_att.yaw= att_t.yaw;
         pub_att.publish(plane_att);
       }
       //IF CURRENT WAYPOINT RECEIVED
       if(msg.msgid == MAVLINK_MSG_ID_MISSION_CURRENT)
       {
         mavlink_mission_current_t current_t;
         mavlink_msg_mission_current_decode(&msg,&current_t);
         wp_current.wp_current= current_t.seq;
         pub_wp_current.publish(wp_current);
       }

     }

     if_mavlink.if_good= if_MavLink;
     pub_if_mavlink.publish(if_mavlink);
     //r.sleep();

   }
}

}//namespace ends
