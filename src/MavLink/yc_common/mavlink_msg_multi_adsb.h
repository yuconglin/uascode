// MESSAGE MULTI_ADSB PACKING

#define MAVLINK_MSG_ID_MULTI_ADSB 207

typedef struct __mavlink_multi_adsb_t
{
 uint32_t addrs[5]; ///< addresses
 float lats[5]; ///< lattitudes
 float lons[5]; ///< longitudes
 float alts[5]; ///< altitudes
 float yaws[5]; ///< yaws or headings
 uint8_t number; ///< number of adsbs
} mavlink_multi_adsb_t;

#define MAVLINK_MSG_ID_MULTI_ADSB_LEN 101
#define MAVLINK_MSG_ID_207_LEN 101

#define MAVLINK_MSG_ID_MULTI_ADSB_CRC 4
#define MAVLINK_MSG_ID_207_CRC 4

#define MAVLINK_MSG_MULTI_ADSB_FIELD_ADDRS_LEN 5
#define MAVLINK_MSG_MULTI_ADSB_FIELD_LATS_LEN 5
#define MAVLINK_MSG_MULTI_ADSB_FIELD_LONS_LEN 5
#define MAVLINK_MSG_MULTI_ADSB_FIELD_ALTS_LEN 5
#define MAVLINK_MSG_MULTI_ADSB_FIELD_YAWS_LEN 5

#define MAVLINK_MESSAGE_INFO_MULTI_ADSB { \
	"MULTI_ADSB", \
	6, \
	{  { "addrs", NULL, MAVLINK_TYPE_UINT32_T, 5, 0, offsetof(mavlink_multi_adsb_t, addrs) }, \
         { "lats", NULL, MAVLINK_TYPE_FLOAT, 5, 20, offsetof(mavlink_multi_adsb_t, lats) }, \
         { "lons", NULL, MAVLINK_TYPE_FLOAT, 5, 40, offsetof(mavlink_multi_adsb_t, lons) }, \
         { "alts", NULL, MAVLINK_TYPE_FLOAT, 5, 60, offsetof(mavlink_multi_adsb_t, alts) }, \
         { "yaws", NULL, MAVLINK_TYPE_FLOAT, 5, 80, offsetof(mavlink_multi_adsb_t, yaws) }, \
         { "number", NULL, MAVLINK_TYPE_UINT8_T, 0, 100, offsetof(mavlink_multi_adsb_t, number) }, \
         } \
}


/**
 * @brief Pack a multi_adsb message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param number number of adsbs
 * @param addrs addresses
 * @param lats lattitudes
 * @param lons longitudes
 * @param alts altitudes
 * @param yaws yaws or headings
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_multi_adsb_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t number, const uint32_t *addrs, const float *lats, const float *lons, const float *alts, const float *yaws)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MULTI_ADSB_LEN];
	_mav_put_uint8_t(buf, 100, number);
	_mav_put_uint32_t_array(buf, 0, addrs, 5);
	_mav_put_float_array(buf, 20, lats, 5);
	_mav_put_float_array(buf, 40, lons, 5);
	_mav_put_float_array(buf, 60, alts, 5);
	_mav_put_float_array(buf, 80, yaws, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MULTI_ADSB_LEN);
#else
	mavlink_multi_adsb_t packet;
	packet.number = number;
	mav_array_memcpy(packet.addrs, addrs, sizeof(uint32_t)*5);
	mav_array_memcpy(packet.lats, lats, sizeof(float)*5);
	mav_array_memcpy(packet.lons, lons, sizeof(float)*5);
	mav_array_memcpy(packet.alts, alts, sizeof(float)*5);
	mav_array_memcpy(packet.yaws, yaws, sizeof(float)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MULTI_ADSB_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MULTI_ADSB;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MULTI_ADSB_LEN, MAVLINK_MSG_ID_MULTI_ADSB_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MULTI_ADSB_LEN);
#endif
}

/**
 * @brief Pack a multi_adsb message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param number number of adsbs
 * @param addrs addresses
 * @param lats lattitudes
 * @param lons longitudes
 * @param alts altitudes
 * @param yaws yaws or headings
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_multi_adsb_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t number,const uint32_t *addrs,const float *lats,const float *lons,const float *alts,const float *yaws)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MULTI_ADSB_LEN];
	_mav_put_uint8_t(buf, 100, number);
	_mav_put_uint32_t_array(buf, 0, addrs, 5);
	_mav_put_float_array(buf, 20, lats, 5);
	_mav_put_float_array(buf, 40, lons, 5);
	_mav_put_float_array(buf, 60, alts, 5);
	_mav_put_float_array(buf, 80, yaws, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MULTI_ADSB_LEN);
#else
	mavlink_multi_adsb_t packet;
	packet.number = number;
	mav_array_memcpy(packet.addrs, addrs, sizeof(uint32_t)*5);
	mav_array_memcpy(packet.lats, lats, sizeof(float)*5);
	mav_array_memcpy(packet.lons, lons, sizeof(float)*5);
	mav_array_memcpy(packet.alts, alts, sizeof(float)*5);
	mav_array_memcpy(packet.yaws, yaws, sizeof(float)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MULTI_ADSB_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MULTI_ADSB;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MULTI_ADSB_LEN, MAVLINK_MSG_ID_MULTI_ADSB_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MULTI_ADSB_LEN);
#endif
}

/**
 * @brief Encode a multi_adsb struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param multi_adsb C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_multi_adsb_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_multi_adsb_t* multi_adsb)
{
	return mavlink_msg_multi_adsb_pack(system_id, component_id, msg, multi_adsb->number, multi_adsb->addrs, multi_adsb->lats, multi_adsb->lons, multi_adsb->alts, multi_adsb->yaws);
}

/**
 * @brief Encode a multi_adsb struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param multi_adsb C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_multi_adsb_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_multi_adsb_t* multi_adsb)
{
	return mavlink_msg_multi_adsb_pack_chan(system_id, component_id, chan, msg, multi_adsb->number, multi_adsb->addrs, multi_adsb->lats, multi_adsb->lons, multi_adsb->alts, multi_adsb->yaws);
}

/**
 * @brief Send a multi_adsb message
 * @param chan MAVLink channel to send the message
 *
 * @param number number of adsbs
 * @param addrs addresses
 * @param lats lattitudes
 * @param lons longitudes
 * @param alts altitudes
 * @param yaws yaws or headings
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_multi_adsb_send(mavlink_channel_t chan, uint8_t number, const uint32_t *addrs, const float *lats, const float *lons, const float *alts, const float *yaws)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MULTI_ADSB_LEN];
	_mav_put_uint8_t(buf, 100, number);
	_mav_put_uint32_t_array(buf, 0, addrs, 5);
	_mav_put_float_array(buf, 20, lats, 5);
	_mav_put_float_array(buf, 40, lons, 5);
	_mav_put_float_array(buf, 60, alts, 5);
	_mav_put_float_array(buf, 80, yaws, 5);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_ADSB, buf, MAVLINK_MSG_ID_MULTI_ADSB_LEN, MAVLINK_MSG_ID_MULTI_ADSB_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_ADSB, buf, MAVLINK_MSG_ID_MULTI_ADSB_LEN);
#endif
#else
	mavlink_multi_adsb_t packet;
	packet.number = number;
	mav_array_memcpy(packet.addrs, addrs, sizeof(uint32_t)*5);
	mav_array_memcpy(packet.lats, lats, sizeof(float)*5);
	mav_array_memcpy(packet.lons, lons, sizeof(float)*5);
	mav_array_memcpy(packet.alts, alts, sizeof(float)*5);
	mav_array_memcpy(packet.yaws, yaws, sizeof(float)*5);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_ADSB, (const char *)&packet, MAVLINK_MSG_ID_MULTI_ADSB_LEN, MAVLINK_MSG_ID_MULTI_ADSB_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MULTI_ADSB, (const char *)&packet, MAVLINK_MSG_ID_MULTI_ADSB_LEN);
#endif
#endif
}

#endif

// MESSAGE MULTI_ADSB UNPACKING


/**
 * @brief Get field number from multi_adsb message
 *
 * @return number of adsbs
 */
static inline uint8_t mavlink_msg_multi_adsb_get_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  100);
}

/**
 * @brief Get field addrs from multi_adsb message
 *
 * @return addresses
 */
static inline uint16_t mavlink_msg_multi_adsb_get_addrs(const mavlink_message_t* msg, uint32_t *addrs)
{
	return _MAV_RETURN_uint32_t_array(msg, addrs, 5,  0);
}

/**
 * @brief Get field lats from multi_adsb message
 *
 * @return lattitudes
 */
static inline uint16_t mavlink_msg_multi_adsb_get_lats(const mavlink_message_t* msg, float *lats)
{
	return _MAV_RETURN_float_array(msg, lats, 5,  20);
}

/**
 * @brief Get field lons from multi_adsb message
 *
 * @return longitudes
 */
static inline uint16_t mavlink_msg_multi_adsb_get_lons(const mavlink_message_t* msg, float *lons)
{
	return _MAV_RETURN_float_array(msg, lons, 5,  40);
}

/**
 * @brief Get field alts from multi_adsb message
 *
 * @return altitudes
 */
static inline uint16_t mavlink_msg_multi_adsb_get_alts(const mavlink_message_t* msg, float *alts)
{
	return _MAV_RETURN_float_array(msg, alts, 5,  60);
}

/**
 * @brief Get field yaws from multi_adsb message
 *
 * @return yaws or headings
 */
static inline uint16_t mavlink_msg_multi_adsb_get_yaws(const mavlink_message_t* msg, float *yaws)
{
	return _MAV_RETURN_float_array(msg, yaws, 5,  80);
}

/**
 * @brief Decode a multi_adsb message into a struct
 *
 * @param msg The message to decode
 * @param multi_adsb C-struct to decode the message contents into
 */
static inline void mavlink_msg_multi_adsb_decode(const mavlink_message_t* msg, mavlink_multi_adsb_t* multi_adsb)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_multi_adsb_get_addrs(msg, multi_adsb->addrs);
	mavlink_msg_multi_adsb_get_lats(msg, multi_adsb->lats);
	mavlink_msg_multi_adsb_get_lons(msg, multi_adsb->lons);
	mavlink_msg_multi_adsb_get_alts(msg, multi_adsb->alts);
	mavlink_msg_multi_adsb_get_yaws(msg, multi_adsb->yaws);
	multi_adsb->number = mavlink_msg_multi_adsb_get_number(msg);
#else
	memcpy(multi_adsb, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MULTI_ADSB_LEN);
#endif
}
