/**
 * @brief MISSION_CURRENT publish plugin
 * @file mission_current.cpp
 * @author Yucong Lin <astroinstrulin@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/UInt16.h>

namespace mavplugin {

class MissionCurrentPubPlugin:public MavRosPlugin{
public:
    MissionCurrentPubPlugin():
        nh("~"),
        uas(nullptr)
    { }

    void initialize(UAS &uas_)
    {
        uas = &uas_;
        mc_pub= nh.advertise<std_msgs::UInt16>("mission_current",10);
    }

    std::string const get_name() const {
        return "MISSION_CURRENT";
    }

    const message_map get_rx_handlers() {
        return{
          MESSAGE_HANDLER(MAVLINK_MSG_ID_MISSION_CURRENT,&MissionCurrentPubPlugin::handle_mission_current)
        };
    }

private:
    UAS *uas;

    ros::NodeHandle nh;
    ros::Publisher mc_pub;

    void handle_mission_current(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid){
        mavlink_mission_current_t m_current;
        mavlink_msg_mission_current_decode(msg, &m_current);

        std_msgs::UInt16Ptr seq = boost::make_shared<std_msgs::UInt16>();
        seq->data= m_current.seq;
        mc_pub.publish(seq);
    }//handle_mission_current

};

} //namespace ends

PLUGINLIB_EXPORT_CLASS(mavplugin::MissionCurrentPubPlugin,mavplugin::MavRosPlugin)
