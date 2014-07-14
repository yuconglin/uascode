// MESSAGE ATTITUDE3 PACKING

#define MAVLINK_MSG_ID_ATTITUDE3 205

typedef struct __mavlink_attitude3_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float roll; ///< Roll angle (rad, -pi..+pi)
 float pitch; ///< Pitch angle (rad, -pi..+pi)
 float yaw; ///< Yaw angle (rad, -pi..+pi)
} mavlink_attitude3_t;

#define MAVLINK_MSG_ID_ATTITUDE3_LEN 16
#define MAVLINK_MSG_ID_205_LEN 16

#define MAVLINK_MSG_ID_ATTITUDE3_CRC 46
#define MAVLINK_MSG_ID_205_CRC 46



#define MAVLINK_MESSAGE_INFO_ATTITUDE3 { \
	"ATTITUDE3", \
	4, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_attitude3_t, time_boot_ms) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_attitude3_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude3_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude3_t, yaw) }, \
         } \
}


/**
 * @brief Pack a attitude3 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude3_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATTITUDE3_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE3_LEN);
#else
	mavlink_attitude3_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE3_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ATTITUDE3;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE3_LEN, MAVLINK_MSG_ID_ATTITUDE3_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE3_LEN);
#endif
}

/**
 * @brief Pack a attitude3 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude3_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,float roll,float pitch,float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATTITUDE3_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE3_LEN);
#else
	mavlink_attitude3_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE3_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ATTITUDE3;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE3_LEN, MAVLINK_MSG_ID_ATTITUDE3_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE3_LEN);
#endif
}

/**
 * @brief Encode a attitude3 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude3_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude3_t* attitude3)
{
	return mavlink_msg_attitude3_pack(system_id, component_id, msg, attitude3->time_boot_ms, attitude3->roll, attitude3->pitch, attitude3->yaw);
}

/**
 * @brief Encode a attitude3 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude3_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude3_t* attitude3)
{
	return mavlink_msg_attitude3_pack_chan(system_id, component_id, chan, msg, attitude3->time_boot_ms, attitude3->roll, attitude3->pitch, attitude3->yaw);
}

/**
 * @brief Send a attitude3 message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param roll Roll angle (rad, -pi..+pi)
 * @param pitch Pitch angle (rad, -pi..+pi)
 * @param yaw Yaw angle (rad, -pi..+pi)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude3_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ATTITUDE3_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, roll);
	_mav_put_float(buf, 8, pitch);
	_mav_put_float(buf, 12, yaw);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE3, buf, MAVLINK_MSG_ID_ATTITUDE3_LEN, MAVLINK_MSG_ID_ATTITUDE3_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE3, buf, MAVLINK_MSG_ID_ATTITUDE3_LEN);
#endif
#else
	mavlink_attitude3_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE3, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE3_LEN, MAVLINK_MSG_ID_ATTITUDE3_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE3, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE3_LEN);
#endif
#endif
}

#endif

// MESSAGE ATTITUDE3 UNPACKING


/**
 * @brief Get field time_boot_ms from attitude3 message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_attitude3_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field roll from attitude3 message
 *
 * @return Roll angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_attitude3_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch from attitude3 message
 *
 * @return Pitch angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_attitude3_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yaw from attitude3 message
 *
 * @return Yaw angle (rad, -pi..+pi)
 */
static inline float mavlink_msg_attitude3_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a attitude3 message into a struct
 *
 * @param msg The message to decode
 * @param attitude3 C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude3_decode(const mavlink_message_t* msg, mavlink_attitude3_t* attitude3)
{
#if MAVLINK_NEED_BYTE_SWAP
	attitude3->time_boot_ms = mavlink_msg_attitude3_get_time_boot_ms(msg);
	attitude3->roll = mavlink_msg_attitude3_get_roll(msg);
	attitude3->pitch = mavlink_msg_attitude3_get_pitch(msg);
	attitude3->yaw = mavlink_msg_attitude3_get_yaw(msg);
#else
	memcpy(attitude3, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ATTITUDE3_LEN);
#endif
}
