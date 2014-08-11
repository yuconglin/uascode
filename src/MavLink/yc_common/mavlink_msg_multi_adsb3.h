// MESSAGE MULTI_ADSB3 PACKING

#define MAVLINK_MSG_ID_MULTI_ADSB3 211

typedef struct __mavlink_multi_adsb3_t
{
 uint32_t addrs[3]; ///< Participant Addrss
 float lats[3]; ///< Latitude
 float lons[3]; ///< Longitude
 float yaws[3]; ///< Track/Heading 
 uint8_t number; ///< Number of adsb obstacles 
} mavlink_multi_adsb3_t;

#define MAVLINK_MSG_ID_MULTI_ADSB3_LEN 49
#define MAVLINK_MSG_ID_211_LEN 49

#define MAVLINK_MSG_ID_MULTI_ADSB3_CRC 156
#define MAVLINK_MSG_ID_211_CRC 156

#define MAVLINK_MSG_MULTI_ADSB3_FIELD_ADDRS_LEN 3
#define MAVLINK_MSG_MULTI_ADSB3_FIELD_LATS_LEN 3
#define MAVLINK_MSG_MULTI_ADSB3_FIELD_LONS_LEN 3
#define MAVLINK_MSG_MULTI_ADSB3_FIELD_YAWS_LEN 3

#define MAVLINK_MESSAGE_INFO_MULTI_ADSB3 { \
	"MULTI_ADSB3", \
	5, \
	{  { "addrs", NULL, MAVLINK_TYPE_UINT32_T, 3, 0, offsetof(mavlink_multi_adsb3_t, addrs) }, \
         { "lats", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_multi_adsb3_t, lats) }, \
         { "lons", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_multi_adsb3_t, lons) }, \
         { "yaws", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_multi_adsb3_t, yaws) }, \
         { "number", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_multi_adsb3_t, number) }, \
         } \
}


/**
 * @brief Pack a multi_adsb3 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param addrs Participant Addrss
 * @param lats Latitude
 * @param lons Longitude
 * @param yaws Track/Heading 
 * @param number Number of adsb obstacles 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_multi_adsb3_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const uint32_t *addrs, const float *lats, const float *lons, const float *yaws, uint8_t number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MULTI_ADSB3_LEN];
	_mav_put_uint8_t(buf, 48, number);
	_mav_put_uint32_t_array(buf, 0, addrs, 3);
	_mav_put_float_array(buf, 12, lats, 3);
	_mav_put_float_array(buf, 24, lons, 3);
	_mav_put_float_array(buf, 36, yaws, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MULTI_ADSB3_LEN);
#else
	mavlink_multi_adsb3_t packet;
	packet.number = number;
	mav_array_memcpy(packet.addrs, addrs, sizeof(uint32_t)*3);
	mav_array_memcpy(packet.lats, lats, sizeof(float)*3);
	mav_array_memcpy(packet.lons, lons, sizeof(float)*3);
	mav_array_memcpy(packet.yaws, yaws, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MULTI_ADSB3_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MULTI_ADSB3;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MULTI_ADSB3_LEN, MAVLINK_MSG_ID_MULTI_ADSB3_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MULTI_ADSB3_LEN);
#endif
}

/**
 * @brief Pack a multi_adsb3 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param addrs Participant Addrss
 * @param lats Latitude
 * @param lons Longitude
 * @param yaws Track/Heading 
 * @param number Number of adsb obstacles 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_multi_adsb3_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const uint32_t *addrs,const float *lats,const float *lons,const float *yaws,uint8_t number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MULTI_ADSB3_LEN];
	_mav_put_uint8_t(buf, 48, number);
	_mav_put_uint32_t_array(buf, 0, addrs, 3);
	_mav_put_float_array(buf, 12, lats, 3);
	_mav_put_float_array(buf, 24, lons, 3);
	_mav_put_float_array(buf, 36, yaws, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MULTI_ADSB3_LEN);
#else
	mavlink_multi_adsb3_t packet;
	packet.number = number;
	mav_array_memcpy(packet.addrs, addrs, sizeof(uint32_t)*3);
	mav_array_memcpy(packet.lats, lats, sizeof(float)*3);
	mav_array_memcpy(packet.lons, lons, sizeof(float)*3);
	mav_array_memcpy(packet.yaws, yaws, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MULTI_ADSB3_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MULTI_ADSB3;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MULTI_ADSB3_LEN, MAVLINK_MSG_ID_MULTI_ADSB3_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MULTI_ADSB3_LEN);
#endif
}

/**
 * @brief Encode a multi_adsb3 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param multi_adsb3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_multi_adsb3_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_multi_adsb3_t* multi_adsb3)
{
	return mavlink_msg_multi_adsb3_pack(system_id, component_id, msg, multi_adsb3->addrs, multi_adsb3->lats, multi_adsb3->lons, multi_adsb3->yaws, multi_adsb3->number);
}

/**
 * @brief Encode a multi_adsb3 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param multi_adsb3 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_multi_adsb3_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_multi_adsb3_t* multi_adsb3)
{
	return mavlink_msg_multi_adsb3_pack_chan(system_id, component_id, chan, msg, multi_adsb3->addrs, multi_adsb3->lats, multi_adsb3->lons, multi_adsb3->yaws, multi_adsb3->number);
}

/**
 * @brief Send a multi_adsb3 message
 * @param chan MAVLink channel to send the message
 *
 * @param addrs Participant Addrss
 * @param lats Latitude
 * @param lons Longitude
 * @param yaws Track/Heading 
 * @param number Number of adsb obstacles 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_multi_adsb3_send(mavlink_channel_t chan, const uint32_t *addrs, const float *lats, const float *lons, const float *yaws, uint8_t number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MULTI_ADSB3_LEN];
	_mav_put_uint8_t(buf, 48, number);
	_mav_put_uint32_t_array(buf, 0, addrs, 3);
	_mav_put_float_array(buf, 12, lats, 3);
	_mav_put_float_array(buf, 24, lons, 3);
	_mav_put_float_array(buf, 36, yaws, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_ADSB3, buf, MAVLINK_MSG_ID_MULTI_ADSB3_LEN, MAVLINK_MSG_ID_MULTI_ADSB3_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_ADSB3, buf, MAVLINK_MSG_ID_MULTI_ADSB3_LEN);
#endif
#else
	mavlink_multi_adsb3_t packet;
	packet.number = number;
	mav_array_memcpy(packet.addrs, addrs, sizeof(uint32_t)*3);
	mav_array_memcpy(packet.lats, lats, sizeof(float)*3);
	mav_array_memcpy(packet.lons, lons, sizeof(float)*3);
	mav_array_memcpy(packet.yaws, yaws, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_ADSB3, (const char *)&packet, MAVLINK_MSG_ID_MULTI_ADSB3_LEN, MAVLINK_MSG_ID_MULTI_ADSB3_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_ADSB3, (const char *)&packet, MAVLINK_MSG_ID_MULTI_ADSB3_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MULTI_ADSB3_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_multi_adsb3_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint32_t *addrs, const float *lats, const float *lons, const float *yaws, uint8_t number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 48, number);
	_mav_put_uint32_t_array(buf, 0, addrs, 3);
	_mav_put_float_array(buf, 12, lats, 3);
	_mav_put_float_array(buf, 24, lons, 3);
	_mav_put_float_array(buf, 36, yaws, 3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_ADSB3, buf, MAVLINK_MSG_ID_MULTI_ADSB3_LEN, MAVLINK_MSG_ID_MULTI_ADSB3_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_ADSB3, buf, MAVLINK_MSG_ID_MULTI_ADSB3_LEN);
#endif
#else
	mavlink_multi_adsb3_t *packet = (mavlink_multi_adsb3_t *)msgbuf;
	packet->number = number;
	mav_array_memcpy(packet->addrs, addrs, sizeof(uint32_t)*3);
	mav_array_memcpy(packet->lats, lats, sizeof(float)*3);
	mav_array_memcpy(packet->lons, lons, sizeof(float)*3);
	mav_array_memcpy(packet->yaws, yaws, sizeof(float)*3);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_ADSB3, (const char *)packet, MAVLINK_MSG_ID_MULTI_ADSB3_LEN, MAVLINK_MSG_ID_MULTI_ADSB3_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_ADSB3, (const char *)packet, MAVLINK_MSG_ID_MULTI_ADSB3_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MULTI_ADSB3 UNPACKING


/**
 * @brief Get field addrs from multi_adsb3 message
 *
 * @return Participant Addrss
 */
static inline uint16_t mavlink_msg_multi_adsb3_get_addrs(const mavlink_message_t* msg, uint32_t *addrs)
{
	return _MAV_RETURN_uint32_t_array(msg, addrs, 3,  0);
}

/**
 * @brief Get field lats from multi_adsb3 message
 *
 * @return Latitude
 */
static inline uint16_t mavlink_msg_multi_adsb3_get_lats(const mavlink_message_t* msg, float *lats)
{
	return _MAV_RETURN_float_array(msg, lats, 3,  12);
}

/**
 * @brief Get field lons from multi_adsb3 message
 *
 * @return Longitude
 */
static inline uint16_t mavlink_msg_multi_adsb3_get_lons(const mavlink_message_t* msg, float *lons)
{
	return _MAV_RETURN_float_array(msg, lons, 3,  24);
}

/**
 * @brief Get field yaws from multi_adsb3 message
 *
 * @return Track/Heading 
 */
static inline uint16_t mavlink_msg_multi_adsb3_get_yaws(const mavlink_message_t* msg, float *yaws)
{
	return _MAV_RETURN_float_array(msg, yaws, 3,  36);
}

/**
 * @brief Get field number from multi_adsb3 message
 *
 * @return Number of adsb obstacles 
 */
static inline uint8_t mavlink_msg_multi_adsb3_get_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Decode a multi_adsb3 message into a struct
 *
 * @param msg The message to decode
 * @param multi_adsb3 C-struct to decode the message contents into
 */
static inline void mavlink_msg_multi_adsb3_decode(const mavlink_message_t* msg, mavlink_multi_adsb3_t* multi_adsb3)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_multi_adsb3_get_addrs(msg, multi_adsb3->addrs);
	mavlink_msg_multi_adsb3_get_lats(msg, multi_adsb3->lats);
	mavlink_msg_multi_adsb3_get_lons(msg, multi_adsb3->lons);
	mavlink_msg_multi_adsb3_get_yaws(msg, multi_adsb3->yaws);
	multi_adsb3->number = mavlink_msg_multi_adsb3_get_number(msg);
#else
	memcpy(multi_adsb3, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MULTI_ADSB3_LEN);
#endif
}
