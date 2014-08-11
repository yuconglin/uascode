// MESSAGE GLOBAL_POSITION_SETPOINT_FLAG_INT PACKING

#define MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT 212

typedef struct __mavlink_global_position_setpoint_flag_int_t
{
 int32_t latitude; ///< Latitude (WGS84), in degrees * 1E7
 int32_t longitude; ///< Longitude (WGS84), in degrees * 1E7
 int32_t altitude; ///< Altitude (WGS84), in meters * 1000 (positive for up)
 int16_t yaw; ///< Desired yaw angle in degrees * 100
 uint8_t coordinate_frame; ///< Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 uint8_t inter_exist; ///< IF An Intermediate waypoint has already been inserted
} mavlink_global_position_setpoint_flag_int_t;

#define MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN 16
#define MAVLINK_MSG_ID_212_LEN 16

#define MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_CRC 81
#define MAVLINK_MSG_ID_212_CRC 81



#define MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_SETPOINT_FLAG_INT { \
	"GLOBAL_POSITION_SETPOINT_FLAG_INT", \
	6, \
	{  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_global_position_setpoint_flag_int_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_global_position_setpoint_flag_int_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_global_position_setpoint_flag_int_t, altitude) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_global_position_setpoint_flag_int_t, yaw) }, \
         { "coordinate_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_global_position_setpoint_flag_int_t, coordinate_frame) }, \
         { "inter_exist", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_global_position_setpoint_flag_int_t, inter_exist) }, \
         } \
}


/**
 * @brief Pack a global_position_setpoint_flag_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param coordinate_frame Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84), in degrees * 1E7
 * @param altitude Altitude (WGS84), in meters * 1000 (positive for up)
 * @param yaw Desired yaw angle in degrees * 100
 * @param inter_exist IF An Intermediate waypoint has already been inserted
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_setpoint_flag_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t coordinate_frame, int32_t latitude, int32_t longitude, int32_t altitude, int16_t yaw, uint8_t inter_exist)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN];
	_mav_put_int32_t(buf, 0, latitude);
	_mav_put_int32_t(buf, 4, longitude);
	_mav_put_int32_t(buf, 8, altitude);
	_mav_put_int16_t(buf, 12, yaw);
	_mav_put_uint8_t(buf, 14, coordinate_frame);
	_mav_put_uint8_t(buf, 15, inter_exist);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN);
#else
	mavlink_global_position_setpoint_flag_int_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.yaw = yaw;
	packet.coordinate_frame = coordinate_frame;
	packet.inter_exist = inter_exist;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN);
#endif
}

/**
 * @brief Pack a global_position_setpoint_flag_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param coordinate_frame Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84), in degrees * 1E7
 * @param altitude Altitude (WGS84), in meters * 1000 (positive for up)
 * @param yaw Desired yaw angle in degrees * 100
 * @param inter_exist IF An Intermediate waypoint has already been inserted
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_global_position_setpoint_flag_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t coordinate_frame,int32_t latitude,int32_t longitude,int32_t altitude,int16_t yaw,uint8_t inter_exist)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN];
	_mav_put_int32_t(buf, 0, latitude);
	_mav_put_int32_t(buf, 4, longitude);
	_mav_put_int32_t(buf, 8, altitude);
	_mav_put_int16_t(buf, 12, yaw);
	_mav_put_uint8_t(buf, 14, coordinate_frame);
	_mav_put_uint8_t(buf, 15, inter_exist);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN);
#else
	mavlink_global_position_setpoint_flag_int_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.yaw = yaw;
	packet.coordinate_frame = coordinate_frame;
	packet.inter_exist = inter_exist;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN);
#endif
}

/**
 * @brief Encode a global_position_setpoint_flag_int struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param global_position_setpoint_flag_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_setpoint_flag_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_global_position_setpoint_flag_int_t* global_position_setpoint_flag_int)
{
	return mavlink_msg_global_position_setpoint_flag_int_pack(system_id, component_id, msg, global_position_setpoint_flag_int->coordinate_frame, global_position_setpoint_flag_int->latitude, global_position_setpoint_flag_int->longitude, global_position_setpoint_flag_int->altitude, global_position_setpoint_flag_int->yaw, global_position_setpoint_flag_int->inter_exist);
}

/**
 * @brief Encode a global_position_setpoint_flag_int struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param global_position_setpoint_flag_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_global_position_setpoint_flag_int_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_global_position_setpoint_flag_int_t* global_position_setpoint_flag_int)
{
	return mavlink_msg_global_position_setpoint_flag_int_pack_chan(system_id, component_id, chan, msg, global_position_setpoint_flag_int->coordinate_frame, global_position_setpoint_flag_int->latitude, global_position_setpoint_flag_int->longitude, global_position_setpoint_flag_int->altitude, global_position_setpoint_flag_int->yaw, global_position_setpoint_flag_int->inter_exist);
}

/**
 * @brief Send a global_position_setpoint_flag_int message
 * @param chan MAVLink channel to send the message
 *
 * @param coordinate_frame Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 * @param latitude Latitude (WGS84), in degrees * 1E7
 * @param longitude Longitude (WGS84), in degrees * 1E7
 * @param altitude Altitude (WGS84), in meters * 1000 (positive for up)
 * @param yaw Desired yaw angle in degrees * 100
 * @param inter_exist IF An Intermediate waypoint has already been inserted
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_global_position_setpoint_flag_int_send(mavlink_channel_t chan, uint8_t coordinate_frame, int32_t latitude, int32_t longitude, int32_t altitude, int16_t yaw, uint8_t inter_exist)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN];
	_mav_put_int32_t(buf, 0, latitude);
	_mav_put_int32_t(buf, 4, longitude);
	_mav_put_int32_t(buf, 8, altitude);
	_mav_put_int16_t(buf, 12, yaw);
	_mav_put_uint8_t(buf, 14, coordinate_frame);
	_mav_put_uint8_t(buf, 15, inter_exist);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN);
#endif
#else
	mavlink_global_position_setpoint_flag_int_t packet;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.yaw = yaw;
	packet.coordinate_frame = coordinate_frame;
	packet.inter_exist = inter_exist;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT, (const char *)&packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_global_position_setpoint_flag_int_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t coordinate_frame, int32_t latitude, int32_t longitude, int32_t altitude, int16_t yaw, uint8_t inter_exist)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, latitude);
	_mav_put_int32_t(buf, 4, longitude);
	_mav_put_int32_t(buf, 8, altitude);
	_mav_put_int16_t(buf, 12, yaw);
	_mav_put_uint8_t(buf, 14, coordinate_frame);
	_mav_put_uint8_t(buf, 15, inter_exist);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT, buf, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN);
#endif
#else
	mavlink_global_position_setpoint_flag_int_t *packet = (mavlink_global_position_setpoint_flag_int_t *)msgbuf;
	packet->latitude = latitude;
	packet->longitude = longitude;
	packet->altitude = altitude;
	packet->yaw = yaw;
	packet->coordinate_frame = coordinate_frame;
	packet->inter_exist = inter_exist;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT, (const char *)packet, MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GLOBAL_POSITION_SETPOINT_FLAG_INT UNPACKING


/**
 * @brief Get field coordinate_frame from global_position_setpoint_flag_int message
 *
 * @return Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT
 */
static inline uint8_t mavlink_msg_global_position_setpoint_flag_int_get_coordinate_frame(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field latitude from global_position_setpoint_flag_int message
 *
 * @return Latitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_global_position_setpoint_flag_int_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from global_position_setpoint_flag_int message
 *
 * @return Longitude (WGS84), in degrees * 1E7
 */
static inline int32_t mavlink_msg_global_position_setpoint_flag_int_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field altitude from global_position_setpoint_flag_int message
 *
 * @return Altitude (WGS84), in meters * 1000 (positive for up)
 */
static inline int32_t mavlink_msg_global_position_setpoint_flag_int_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field yaw from global_position_setpoint_flag_int message
 *
 * @return Desired yaw angle in degrees * 100
 */
static inline int16_t mavlink_msg_global_position_setpoint_flag_int_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field inter_exist from global_position_setpoint_flag_int message
 *
 * @return IF An Intermediate waypoint has already been inserted
 */
static inline uint8_t mavlink_msg_global_position_setpoint_flag_int_get_inter_exist(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Decode a global_position_setpoint_flag_int message into a struct
 *
 * @param msg The message to decode
 * @param global_position_setpoint_flag_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_global_position_setpoint_flag_int_decode(const mavlink_message_t* msg, mavlink_global_position_setpoint_flag_int_t* global_position_setpoint_flag_int)
{
#if MAVLINK_NEED_BYTE_SWAP
	global_position_setpoint_flag_int->latitude = mavlink_msg_global_position_setpoint_flag_int_get_latitude(msg);
	global_position_setpoint_flag_int->longitude = mavlink_msg_global_position_setpoint_flag_int_get_longitude(msg);
	global_position_setpoint_flag_int->altitude = mavlink_msg_global_position_setpoint_flag_int_get_altitude(msg);
	global_position_setpoint_flag_int->yaw = mavlink_msg_global_position_setpoint_flag_int_get_yaw(msg);
	global_position_setpoint_flag_int->coordinate_frame = mavlink_msg_global_position_setpoint_flag_int_get_coordinate_frame(msg);
	global_position_setpoint_flag_int->inter_exist = mavlink_msg_global_position_setpoint_flag_int_get_inter_exist(msg);
#else
	memcpy(global_position_setpoint_flag_int, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GLOBAL_POSITION_SETPOINT_FLAG_INT_LEN);
#endif
}
