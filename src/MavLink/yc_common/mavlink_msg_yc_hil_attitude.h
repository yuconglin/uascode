// MESSAGE YC_HIL_ATTITUDE PACKING
#include "stdio.h"

#define MAVLINK_MSG_ID_YC_HIL_ATTITUDE 204

typedef struct __mavlink_yc_hil_attitude_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float roll; ///< Roll angle (rad, -pi..+pi)
 float pitch; ///< Pitch angle (rad, -pi..+pi)
 float yaw; ///< Yaw angle (rad, -pi..+pi)
 float rollspeed; ///< Roll angular speed (rad/s)
 float pitchspeed; ///< Pitch angular speed (rad/s)
 float yawspeed; ///< Yaw angular speed (rad/s)
} mavlink_yc_hil_attitude_t;

#define MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN 28
#define MAVLINK_MSG_ID_204_LEN 28

#define MAVLINK_MSG_ID_YC_HIL_ATTITUDE_CRC 131
#define MAVLINK_MSG_ID_204_CRC 131



#define MAVLINK_MESSAGE_INFO_YC_HIL_ATTITUDE { \
	"YC_HIL_ATTITUDE", \
	7, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_yc_hil_attitude_t, time_boot_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_yc_hil_attitude_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_yc_hil_attitude_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_yc_hil_attitude_t, yaw) }, \
         { "rollspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_yc_hil_attitude_t, rollspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_yc_hil_attitude_t, pitchspeed) }, \
         { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_yc_hil_attitude_t, yawspeed) }, \
         } \
}


/**
 * @brief Pack a yc_hil_attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_yc_hil_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, rollspeed);
	_mav_put_float(buf, 20, pitchspeed);
	_mav_put_float(buf, 24, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN);
#else
	mavlink_yc_hil_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_YC_HIL_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN);
#endif
}

/**
 * @brief Pack a yc_hil_attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_yc_hil_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,float roll,float pitch,float yaw,float rollspeed,float pitchspeed,float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, rollspeed);
	_mav_put_float(buf, 20, pitchspeed);
	_mav_put_float(buf, 24, yawspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN);
#else
	mavlink_yc_hil_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_YC_HIL_ATTITUDE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN);
#endif
}

/**
 * @brief Encode a yc_hil_attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param yc_hil_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_yc_hil_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_yc_hil_attitude_t* yc_hil_attitude)
{
	return mavlink_msg_yc_hil_attitude_pack(system_id, component_id, msg, yc_hil_attitude->time_boot_ms, yc_hil_attitude->roll, yc_hil_attitude->pitch, yc_hil_attitude->yaw, yc_hil_attitude->rollspeed, yc_hil_attitude->pitchspeed, yc_hil_attitude->yawspeed);
}

/**
 * @brief Encode a yc_hil_attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param yc_hil_attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_yc_hil_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_yc_hil_attitude_t* yc_hil_attitude)
{
	return mavlink_msg_yc_hil_attitude_pack_chan(system_id, component_id, chan, msg, yc_hil_attitude->time_boot_ms, yc_hil_attitude->roll, yc_hil_attitude->pitch, yc_hil_attitude->yaw, yc_hil_attitude->rollspeed, yc_hil_attitude->pitchspeed, yc_hil_attitude->yawspeed);
}

/**
 * @brief Send a yc_hil_attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @param rollspeed Roll angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @param yawspeed Yaw angular speed (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_yc_hil_attitude_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);
	_mav_put_float(buf, 16, rollspeed);
	_mav_put_float(buf, 20, pitchspeed);
	_mav_put_float(buf, 24, yawspeed);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YC_HIL_ATTITUDE, buf, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YC_HIL_ATTITUDE, buf, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN);
#endif
#else
	mavlink_yc_hil_attitude_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;
	packet.rollspeed = rollspeed;
	packet.pitchspeed = pitchspeed;
	packet.yawspeed = yawspeed;
	//printf("in %.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",roll,pitch,yaw,rollspeed,pitchspeed,yawspeed);
#if MAVLINK_CRC_EXTRA
    //printf("crc extra\n");
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YC_HIL_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YC_HIL_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN);
#endif
#endif
}

#endif

// MESSAGE YC_HIL_ATTITUDE UNPACKING


/**
 * @brief Get field time_boot_ms from yc_hil_attitude message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_yc_hil_attitude_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field roll from yc_hil_attitude message
 *
 * @return Roll angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_yc_hil_attitude_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch from yc_hil_attitude message
 *
 * @return Pitch angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_yc_hil_attitude_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from yc_hil_attitude message
 *
 * @return Yaw angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_yc_hil_attitude_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field rollspeed from yc_hil_attitude message
 *
 * @return Roll angular speed (rad/s)
 */
static inline float mavlink_msg_yc_hil_attitude_get_rollspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pitchspeed from yc_hil_attitude message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_yc_hil_attitude_get_pitchspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field yawspeed from yc_hil_attitude message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_yc_hil_attitude_get_yawspeed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a yc_hil_attitude message into a struct
 *
 * @param msg The message to decode
 * @param yc_hil_attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_yc_hil_attitude_decode(const mavlink_message_t* msg, mavlink_yc_hil_attitude_t* yc_hil_attitude)
{
#if MAVLINK_NEED_BYTE_SWAP
	yc_hil_attitude->time_boot_ms = mavlink_msg_yc_hil_attitude_get_time_boot_ms(msg);
	yc_hil_attitude->roll = mavlink_msg_yc_hil_attitude_get_roll(msg);
	yc_hil_attitude->pitch = mavlink_msg_yc_hil_attitude_get_pitch(msg);
	yc_hil_attitude->yaw = mavlink_msg_yc_hil_attitude_get_yaw(msg);
	yc_hil_attitude->rollspeed = mavlink_msg_yc_hil_attitude_get_rollspeed(msg);
	yc_hil_attitude->pitchspeed = mavlink_msg_yc_hil_attitude_get_pitchspeed(msg);
	yc_hil_attitude->yawspeed = mavlink_msg_yc_hil_attitude_get_yawspeed(msg);
#else
	memcpy(yc_hil_attitude, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_YC_HIL_ATTITUDE_LEN);
#endif
}
