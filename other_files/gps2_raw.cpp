/**
 * @brief GPS2_RAW publish plugin
 * @file gps2_raw.cpp
 * @author Yucong Lin <astroinstrulin@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
#include <angles/angles.h>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/Float64.h>

namespace mavplugin {

class GPSInfo:public diagnostic_updater::DiagnosticTask
{
public:
    explicit GPSInfo(const std::string name) :
        diagnostic_updater::DiagnosticTask(name),
        satellites_visible(-1),
        fix_type(0),
        eph(UINT16_MAX),
        epv(UINT16_MAX)
    { };

    void set_gps2_raw(mavlink_gps2_raw_t &gps) {
        satellites_visible = gps.satellites_visible;
        fix_type = gps.fix_type;
        eph = gps.eph;
        epv = gps.epv;
    }

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
        const int satellites_visible_ = satellites_visible;
        const int fix_type_ = fix_type;
        const uint16_t eph_ = eph;
        const uint16_t epv_ = epv;

        if (satellites_visible_ < 0)
            stat.summary(2, "No satellites");
        else if (fix_type_ < 2 || fix_type_ > 3)
            stat.summary(1, "No fix");
        else if (fix_type_ == 2)
            stat.summary(0, "2D fix");
        else if (fix_type_ == 3)
            stat.summary(0, "3D fix");

        stat.addf("Satellites visible", "%zd", satellites_visible_);
        stat.addf("Fix type", "%d", fix_type_);

        // EPH in centimeters
        if (eph_ != UINT16_MAX)
            stat.addf("EPH (m)", "%.2f", eph_ / 1E2F);
        else
            stat.add("EPH (m)", "Unknown");

        // EPV in centimeters
        if (epv_ != UINT16_MAX)
            stat.addf("EPV (m)", "%.2f", epv_ / 1E2F);
        else
            stat.add("EPV (m)", "Unknown");
    }

private:
    std::atomic<int> satellites_visible;
    std::atomic<int> fix_type;
    std::atomic<uint16_t> eph;
    std::atomic<uint16_t> epv;
};

/**
 * @brief GPS2_RAW plugin
 */
class GPS2Plugin:public MavRosPlugin{
public:
    GPS2Plugin() :
        uas(nullptr),
        gps2_diag("GPS2_RAW")
    {}

    void initialize(UAS &uas_, ros::NodeHandle &nh, diagnostic_updater::Updater &diag_updater)
    {
        uas = &uas_;

        nh.param<std::string>("gps2/frame_id", frame_id, "gps2");
        nh.param<std::string>("gps2/time_ref_source", time_ref_source, frame_id);

        diag_updater.add(gps2_diag);

        fix2_pub= nh.advertise<sensor_msgs::NavSatFix>("fix2",10);
        time2_ref_pub = nh.advertise<sensor_msgs::TimeReference>("time2_reference", 10);
        vel2_pub= nh.advertise<std_msgs::Float64>("gps2_vel",10);
        hdg2_pub= nh.advertise<std_msgs::Float64>("gps2_hdg",10);
    }

    std::string const get_name() const {
        return "GPS2_RAW";
    }

    const message_map get_rx_handlers() {
        return {
            MESSAGE_HANDLER(MAVLINK_MSG_ID_GPS2_RAW, &GPS2Plugin::handle_gps2_raw),
            MESSAGE_HANDLER(MAVLINK_MSG_ID_GPS_STATUS, &GPS2Plugin::handle_gps2_status),
        };
    }

 private:
    UAS *uas;
    std::string frame_id;
    std::string time_ref_source;

    GPSInfo gps2_diag;

    ros::Publisher fix2_pub;
    ros::Publisher time2_ref_pub;
    ros::Publisher vel2_pub;
    ros::Publisher hdg2_pub;

    void handle_gps2_raw(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        mavlink_gps2_raw_t raw_gps2;
        mavlink_msg_gps2_raw_decode(msg, &raw_gps2);

        sensor_msgs::NavSatFixPtr fix = boost::make_shared<sensor_msgs::NavSatFix>();
        std_msgs::Float64Ptr gps2_speed = boost::make_shared<std_msgs::Float64>();
        std_msgs::Float64Ptr gps2_heading = boost::make_shared<std_msgs::Float64>();

        gps2_diag.set_gps2_raw(raw_gps2);
        if (raw_gps2.fix_type < 2) {
            ROS_WARN_THROTTLE_NAMED(60, "gps", "GPS: no fix");
            return;
        }

        fix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        if (raw_gps2.fix_type == 2 || raw_gps2.fix_type == 3)
            fix->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        else
            fix->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;

        fix->latitude = raw_gps2.lat / 1E7; //deg
        fix->longitude = raw_gps2.lon / 1E7; //deg
        fix->altitude = raw_gps2.alt / 1E3; //m

        if (raw_gps2.eph != UINT16_MAX) {
            double hdop = raw_gps2.eph / 1E2;
            double hdop2 = std::pow(hdop, 2);

            // TODO: Check
            // From nmea_navsat_driver
            fix->position_covariance[0] = hdop2;
            fix->position_covariance[4] = hdop2;
            fix->position_covariance[8] = std::pow(2 * hdop, 2);
            fix->position_covariance_type =
                sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        }
        else {
            fix->position_covariance_type =
                sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        }

        fix->header.frame_id = frame_id;
        fix->header.stamp = ros::Time::now();

        // store GPS data in UAS
        double eph = (raw_gps2.eph != UINT16_MAX)? raw_gps2.eph / 1E2 : NAN;
        double epv = (raw_gps2.epv != UINT16_MAX)? raw_gps2.epv / 1E2 : NAN;
        uas->set_gps_llae(fix->latitude, fix->longitude, fix->altitude, eph, epv);
        uas->set_gps_status(fix->status.status == sensor_msgs::NavSatStatus::STATUS_FIX);

        fix2_pub.publish(fix);

        if(raw_gps2.vel != UINT16_MAX && raw_gps2.cog!= UINT16_MAX){
          gps2_speed->data= raw_gps2.vel / 1E2;
          vel2_pub.publish(gps2_speed);
          gps2_heading->data= angles::from_degrees(raw_gps2.cog/1E2);
          hdg2_pub.publish(gps2_heading);
        }
    }//handle_gps2_raw

    void handle_gps2_status(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        mavlink_gps_status_t gps_stat;
        mavlink_msg_gps_status_decode(msg, &gps_stat);

        ROS_INFO_THROTTLE_NAMED(30, "gps", "GPS stat sat visible: %d", gps_stat.satellites_visible);
    }

};

}//namespace ends

PLUGINLIB_EXPORT_CLASS(mavplugin::GPS2Plugin, mavplugin::MavRosPlugin)
