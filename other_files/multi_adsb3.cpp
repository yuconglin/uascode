#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <yucong_rosmsg/MultiObsMsg2.h>

namespace mavplugin {
  class MultiAdsb3Plugin:public MavRosPlugin{
  public:
      MultiAdsb3Plugin():
          uas(nullptr)
      {}

      void initialize(UAS &uas_,
                      ros::NodeHandle &nh,
                      diagnostic_updater::Updater &diag_updater)
      {
          uas = &uas_;
          //or the topic may be just "multi_obstacles"
          adsb3_sub = nh.subscribe("multi_obstacles",10,&MultiAdsb3Plugin::adsb3_cb,this);
      }

      const std::string get_name() const {
          return "MULTI_ADSB3";
      }

      const message_map get_rx_handlers() {
          return {/*Rx disabled */ };
      }

  private:
      UAS* uas;

      ros::NodeHandle adsb3_nh;
      ros::Subscriber adsb3_sub;

      void send_obss3(const yucong_rosmsg::MultiObsMsg2& obss)
      {
          mavlink_multi_adsb3_t adsb_obss;
          int size= obss.MultiObs.size();
          adsb_obss.number= size>3 ? 3:size;

          for(int i=0;i!=adsb_obss.number;++i)
          {
              adsb_obss.addrs[i]= obss.MultiObs[i].address;
              adsb_obss.lats[i]= obss.MultiObs[i].lat;
              adsb_obss.lons[i]= obss.MultiObs[i].lon;
              double hd= M_PI/2- obss.MultiObs[i].head_xy;
              if(hd<0)
                  hd+= 2*M_PI;
              hd= hd*180./M_PI;
              adsb_obss.yaws[i]= hd;
          }//for ends

          //set not set ones to zero
          for(int i= adsb_obss.number;i!=3;++i)
          {
             adsb_obss.addrs[i]=0;
             adsb_obss.lats[i]=0.;
             adsb_obss.lons[i]=0.;
             adsb_obss.yaws[i]=0.;
          }

          mavlink_message_t msg;
          mavlink_msg_multi_adsb3_encode_chan(UAS_PACK_CHAN(uas), &msg, &adsb_obss);
          UAS_FCU(uas)->send_message(&msg);
      }

      void adsb3_cb(const yucong_rosmsg::MultiObsMsg2::ConstPtr &msg) {
          std::cout <<"adsb3_cb" << "/n";
          yucong_rosmsg::MultiObsMsg2 obss_msg2;
          obss_msg2.MultiObs= msg->MultiObs;
          send_obss3(obss_msg2);
      }
  };
}//namespace ends
PLUGINLIB_EXPORT_CLASS(mavplugin::MultiAdsb3Plugin, mavplugin::MavRosPlugin)

