// MESSAGE ADSB_SHORT PACKING

#define MAVLINK_MSG_ID_ADSB_SHORT 203

typedef struct __mavlink_adsb_short_t
{
 uint32_t address; ///< Participant Addrss
 float latitude; ///< Latitude
 float longitude; ///< Longitude
 float altitude; ///< Altitude:actual= ddd*25-1000
 float h_velocity; ///< Horizontal velocity in knots
 float v_velocity; ///< Vertical velocity in 64fpm
 float heading; ///< Track/Heading 
} mavlink_adsb_short_t;

#define MAVLINK_MSG_ID_ADSB_SHORT_LEN 28
#define MAVLINK_MSG_ID_203_LEN 28

#define MAVLINK_MSG_ID_ADSB_SHORT_CRC 211
#define MAVLINK_MSG_ID_203_CRC 211



#define MAVLINK_MESSAGE_INFO_ADSB_SHORT { \
	"ADSB_SHORT", \
	7, \
	{  { "address", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_adsb_short_t, address) }, \
         { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_adsb_short_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_adsb_short_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_adsb_short_t, altitude) }, \
         { "h_velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_adsb_short_t, h_velocity) }, \
         { "v_velocity", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_adsb_short_t, v_velocity) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_adsb_short_t, heading) }, \
         } \
}


/**
 * @brief Pack a adsb_short message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param address Participant Addrss
 * @param latitude Latitude
 * @param longitude Longitude
 * @param altitude Altitude:actual= ddd*25-1000
 * @param h_velocity Horizontal velocity in knots
 * @param v_velocity Vertical velocity in 64fpm
 * @param heading Track/Heading 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adsb_short_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t address, float latitude, float longitude, float altitude, float h_velocity, float v_velocity, float heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_SHORT_LEN];
	_mav_put_uint32_t(buf, 0, address);
	_mav_put_float(buf, 4, latitude);
	_mav_put_float(buf, 8, longitude);
	_mav_put_float(buf, 12, altitude);
	_mav_put_float(buf, 16, h_velocity);
	_mav_put_float(buf, 20, v_velocity);
	_mav_put_float(buf, 24, heading);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADSB_SHORT_LEN);
#else
	mavlink_adsb_short_t packet;
	packet.address = address;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.h_velocity = h_velocity;
	packet.v_velocity = v_velocity;
	packet.heading = heading;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADSB_SHORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADSB_SHORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADSB_SHORT_LEN, MAVLINK_MSG_ID_ADSB_SHORT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADSB_SHORT_LEN);
#endif
}

/**
 * @brief Pack a adsb_short message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param address Participant Addrss
 * @param latitude Latitude
 * @param longitude Longitude
 * @param altitude Altitude:actual= ddd*25-1000
 * @param h_velocity Horizontal velocity in knots
 * @param v_velocity Vertical velocity in 64fpm
 * @param heading Track/Heading 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adsb_short_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t address,float latitude,float longitude,float altitude,float h_velocity,float v_velocity,float heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_SHORT_LEN];
	_mav_put_uint32_t(buf, 0, address);
	_mav_put_float(buf, 4, latitude);
	_mav_put_float(buf, 8, longitude);
	_mav_put_float(buf, 12, altitude);
	_mav_put_float(buf, 16, h_velocity);
	_mav_put_float(buf, 20, v_velocity);
	_mav_put_float(buf, 24, heading);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADSB_SHORT_LEN);
#else
	mavlink_adsb_short_t packet;
	packet.address = address;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.h_velocity = h_velocity;
	packet.v_velocity = v_velocity;
	packet.heading = heading;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADSB_SHORT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADSB_SHORT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADSB_SHORT_LEN, MAVLINK_MSG_ID_ADSB_SHORT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADSB_SHORT_LEN);
#endif
}

/**
 * @brief Encode a adsb_short struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adsb_short C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adsb_short_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adsb_short_t* adsb_short)
{
	return mavlink_msg_adsb_short_pack(system_id, component_id, msg, adsb_short->address, adsb_short->latitude, adsb_short->longitude, adsb_short->altitude, adsb_short->h_velocity, adsb_short->v_velocity, adsb_short->heading);
}

/**
 * @brief Encode a adsb_short struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adsb_short C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adsb_short_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adsb_short_t* adsb_short)
{
	return mavlink_msg_adsb_short_pack_chan(system_id, component_id, chan, msg, adsb_short->address, adsb_short->latitude, adsb_short->longitude, adsb_short->altitude, adsb_short->h_velocity, adsb_short->v_velocity, adsb_short->heading);
}

/**
 * @brief Send a adsb_short message
 * @param chan MAVLink channel to send the message
 *
 * @param address Participant Addrss
 * @param latitude Latitude
 * @param longitude Longitude
 * @param altitude Altitude:actual= ddd*25-1000
 * @param h_velocity Horizontal velocity in knots
 * @param v_velocity Vertical velocity in 64fpm
 * @param heading Track/Heading 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adsb_short_send(mavlink_channel_t chan, uint32_t address, float latitude, float longitude, float altitude, float h_velocity, float v_velocity, float heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_SHORT_LEN];
	_mav_put_uint32_t(buf, 0, address);
	_mav_put_float(buf, 4, latitude);
	_mav_put_float(buf, 8, longitude);
	_mav_put_float(buf, 12, altitude);
	_mav_put_float(buf, 16, h_velocity);
	_mav_put_float(buf, 20, v_velocity);
	_mav_put_float(buf, 24, heading);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_SHORT, buf, MAVLINK_MSG_ID_ADSB_SHORT_LEN, MAVLINK_MSG_ID_ADSB_SHORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_SHORT, buf, MAVLINK_MSG_ID_ADSB_SHORT_LEN);
#endif
#else
	mavlink_adsb_short_t packet;
	packet.address = address;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.h_velocity = h_velocity;
	packet.v_velocity = v_velocity;
	packet.heading = heading;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_SHORT, (const char *)&packet, MAVLINK_MSG_ID_ADSB_SHORT_LEN, MAVLINK_MSG_ID_ADSB_SHORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_SHORT, (const char *)&packet, MAVLINK_MSG_ID_ADSB_SHORT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ADSB_SHORT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_adsb_short_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t address, float latitude, float longitude, float altitude, float h_velocity, float v_velocity, float heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, address);
	_mav_put_float(buf, 4, latitude);
	_mav_put_float(buf, 8, longitude);
	_mav_put_float(buf, 12, altitude);
	_mav_put_float(buf, 16, h_velocity);
	_mav_put_float(buf, 20, v_velocity);
	_mav_put_float(buf, 24, heading);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_SHORT, buf, MAVLINK_MSG_ID_ADSB_SHORT_LEN, MAVLINK_MSG_ID_ADSB_SHORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_SHORT, buf, MAVLINK_MSG_ID_ADSB_SHORT_LEN);
#endif
#else
	mavlink_adsb_short_t *packet = (mavlink_adsb_short_t *)msgbuf;
	packet->address = address;
	packet->latitude = latitude;
	packet->longitude = longitude;
	packet->altitude = altitude;
	packet->h_velocity = h_velocity;
	packet->v_velocity = v_velocity;
	packet->heading = heading;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_SHORT, (const char *)packet, MAVLINK_MSG_ID_ADSB_SHORT_LEN, MAVLINK_MSG_ID_ADSB_SHORT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_SHORT, (const char *)packet, MAVLINK_MSG_ID_ADSB_SHORT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ADSB_SHORT UNPACKING


/**
 * @brief Get field address from adsb_short message
 *
 * @return Participant Addrss
 */
static inline uint32_t mavlink_msg_adsb_short_get_address(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field latitude from adsb_short message
 *
 * @return Latitude
 */
static inline float mavlink_msg_adsb_short_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field longitude from adsb_short message
 *
 * @return Longitude
 */
static inline float mavlink_msg_adsb_short_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field altitude from adsb_short message
 *
 * @return Altitude:actual= ddd*25-1000
 */
static inline float mavlink_msg_adsb_short_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field h_velocity from adsb_short message
 *
 * @return Horizontal velocity in knots
 */
static inline float mavlink_msg_adsb_short_get_h_velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field v_velocity from adsb_short message
 *
 * @return Vertical velocity in 64fpm
 */
static inline float mavlink_msg_adsb_short_get_v_velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field heading from adsb_short message
 *
 * @return Track/Heading 
 */
static inline float mavlink_msg_adsb_short_get_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a adsb_short message into a struct
 *
 * @param msg The message to decode
 * @param adsb_short C-struct to decode the message contents into
 */
static inline void mavlink_msg_adsb_short_decode(const mavlink_message_t* msg, mavlink_adsb_short_t* adsb_short)
{
#if MAVLINK_NEED_BYTE_SWAP
	adsb_short->address = mavlink_msg_adsb_short_get_address(msg);
	adsb_short->latitude = mavlink_msg_adsb_short_get_latitude(msg);
	adsb_short->longitude = mavlink_msg_adsb_short_get_longitude(msg);
	adsb_short->altitude = mavlink_msg_adsb_short_get_altitude(msg);
	adsb_short->h_velocity = mavlink_msg_adsb_short_get_h_velocity(msg);
	adsb_short->v_velocity = mavlink_msg_adsb_short_get_v_velocity(msg);
	adsb_short->heading = mavlink_msg_adsb_short_get_heading(msg);
#else
	memcpy(adsb_short, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ADSB_SHORT_LEN);
#endif
}
