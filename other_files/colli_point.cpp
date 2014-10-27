#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <yucong_rosmsg/ColliPoint.h>

namespace mavplugin {
   class ColliPointPlugin:public MavRosPlugin{
   public:
       ColliPointPlugin():
           uas(nullptr)
       {}

       void initialize(UAS &uas_, ros::NodeHandle &nh, diagnostic_updater::Updater &diag_updater)
       {
          uas = &uas_;
          colli_sub = nh.subscribe("colli_point",10,&ColliPointPlugin::colli_cb,this);
       }

       const std::string get_name() const {
           return "COLLI_POINT";
       }

       const message_map get_rx_handlers() {
           return {/*Rx disabled */};
       }

   private:
       UAS* uas;

       ros::NodeHandle colli_nh;
       ros::Subscriber colli_sub;

       void send_colli_point(const yucong_rosmsg::ColliPoint& colli_pt)
       {
           mavlink_colli_point_t colli_t;
           colli_t.lat= colli_pt.lat;
           colli_t.lon= colli_pt.lon;
           colli_t.alt= colli_pt.alt;

           mavlink_message_t msg;
           mavlink_msg_colli_point_encode_chan(UAS_PACK_CHAN(uas), &msg, &colli_t);
           UAS_FCU(uas)->send_message(&msg);
       }

       void colli_cb(const yucong_rosmsg::ColliPoint::ConstPtr &msg)  {
           yucong_rosmsg::ColliPoint colli_pt;
           colli_pt.lat= msg->lat;
           colli_pt.lon= msg->lon;
           colli_pt.alt= msg->alt;

           send_colli_point(colli_pt);
       }

   };
}//namespace ends

PLUGINLIB_EXPORT_CLASS(mavplugin::ColliPointPlugin, mavplugin::MavRosPlugin)
