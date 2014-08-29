// MESSAGE COLLI_POINT PACKING

#define MAVLINK_MSG_ID_COLLI_POINT 213

typedef struct __mavlink_colli_point_t
{
 float lat; ///< Latitude
 float lon; ///< Longitude
 float alt; ///< Altitude 
} mavlink_colli_point_t;

#define MAVLINK_MSG_ID_COLLI_POINT_LEN 12
#define MAVLINK_MSG_ID_213_LEN 12

#define MAVLINK_MSG_ID_COLLI_POINT_CRC 188
#define MAVLINK_MSG_ID_213_CRC 188



#define MAVLINK_MESSAGE_INFO_COLLI_POINT { \
	"COLLI_POINT", \
	3, \
	{  { "lat", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_colli_point_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_colli_point_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_colli_point_t, alt) }, \
         } \
}


/**
 * @brief Pack a colli_point message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat Latitude
 * @param lon Longitude
 * @param alt Altitude 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_colli_point_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float lat, float lon, float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_COLLI_POINT_LEN];
	_mav_put_float(buf, 0, lat);
	_mav_put_float(buf, 4, lon);
	_mav_put_float(buf, 8, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COLLI_POINT_LEN);
#else
	mavlink_colli_point_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COLLI_POINT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_COLLI_POINT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COLLI_POINT_LEN, MAVLINK_MSG_ID_COLLI_POINT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COLLI_POINT_LEN);
#endif
}

/**
 * @brief Pack a colli_point message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat Latitude
 * @param lon Longitude
 * @param alt Altitude 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_colli_point_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float lat,float lon,float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_COLLI_POINT_LEN];
	_mav_put_float(buf, 0, lat);
	_mav_put_float(buf, 4, lon);
	_mav_put_float(buf, 8, alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_COLLI_POINT_LEN);
#else
	mavlink_colli_point_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COLLI_POINT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_COLLI_POINT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COLLI_POINT_LEN, MAVLINK_MSG_ID_COLLI_POINT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_COLLI_POINT_LEN);
#endif
}

/**
 * @brief Encode a colli_point struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param colli_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_colli_point_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_colli_point_t* colli_point)
{
	return mavlink_msg_colli_point_pack(system_id, component_id, msg, colli_point->lat, colli_point->lon, colli_point->alt);
}

/**
 * @brief Encode a colli_point struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param colli_point C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_colli_point_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_colli_point_t* colli_point)
{
	return mavlink_msg_colli_point_pack_chan(system_id, component_id, chan, msg, colli_point->lat, colli_point->lon, colli_point->alt);
}

/**
 * @brief Send a colli_point message
 * @param chan MAVLink channel to send the message
 *
 * @param lat Latitude
 * @param lon Longitude
 * @param alt Altitude 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_colli_point_send(mavlink_channel_t chan, float lat, float lon, float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_COLLI_POINT_LEN];
	_mav_put_float(buf, 0, lat);
	_mav_put_float(buf, 4, lon);
	_mav_put_float(buf, 8, alt);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COLLI_POINT, buf, MAVLINK_MSG_ID_COLLI_POINT_LEN, MAVLINK_MSG_ID_COLLI_POINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COLLI_POINT, buf, MAVLINK_MSG_ID_COLLI_POINT_LEN);
#endif
#else
	mavlink_colli_point_t packet;
	packet.lat = lat;
	packet.lon = lon;
	packet.alt = alt;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COLLI_POINT, (const char *)&packet, MAVLINK_MSG_ID_COLLI_POINT_LEN, MAVLINK_MSG_ID_COLLI_POINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COLLI_POINT, (const char *)&packet, MAVLINK_MSG_ID_COLLI_POINT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_COLLI_POINT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_colli_point_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float lat, float lon, float alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, lat);
	_mav_put_float(buf, 4, lon);
	_mav_put_float(buf, 8, alt);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COLLI_POINT, buf, MAVLINK_MSG_ID_COLLI_POINT_LEN, MAVLINK_MSG_ID_COLLI_POINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COLLI_POINT, buf, MAVLINK_MSG_ID_COLLI_POINT_LEN);
#endif
#else
	mavlink_colli_point_t *packet = (mavlink_colli_point_t *)msgbuf;
	packet->lat = lat;
	packet->lon = lon;
	packet->alt = alt;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COLLI_POINT, (const char *)packet, MAVLINK_MSG_ID_COLLI_POINT_LEN, MAVLINK_MSG_ID_COLLI_POINT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_COLLI_POINT, (const char *)packet, MAVLINK_MSG_ID_COLLI_POINT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE COLLI_POINT UNPACKING


/**
 * @brief Get field lat from colli_point message
 *
 * @return Latitude
 */
static inline float mavlink_msg_colli_point_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field lon from colli_point message
 *
 * @return Longitude
 */
static inline float mavlink_msg_colli_point_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field alt from colli_point message
 *
 * @return Altitude 
 */
static inline float mavlink_msg_colli_point_get_alt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a colli_point message into a struct
 *
 * @param msg The message to decode
 * @param colli_point C-struct to decode the message contents into
 */
static inline void mavlink_msg_colli_point_decode(const mavlink_message_t* msg, mavlink_colli_point_t* colli_point)
{
#if MAVLINK_NEED_BYTE_SWAP
	colli_point->lat = mavlink_msg_colli_point_get_lat(msg);
	colli_point->lon = mavlink_msg_colli_point_get_lon(msg);
	colli_point->alt = mavlink_msg_colli_point_get_alt(msg);
#else
	memcpy(colli_point, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_COLLI_POINT_LEN);
#endif
}
