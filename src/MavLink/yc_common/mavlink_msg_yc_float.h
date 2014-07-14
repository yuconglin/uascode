// MESSAGE YC_FLOAT PACKING

#define MAVLINK_MSG_ID_YC_FLOAT 206

typedef struct __mavlink_yc_float_t
{
 float data; ///< the float data
} mavlink_yc_float_t;

#define MAVLINK_MSG_ID_YC_FLOAT_LEN 4
#define MAVLINK_MSG_ID_206_LEN 4

#define MAVLINK_MSG_ID_YC_FLOAT_CRC 183
#define MAVLINK_MSG_ID_206_CRC 183



#define MAVLINK_MESSAGE_INFO_YC_FLOAT { \
	"YC_FLOAT", \
	1, \
	{  { "data", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_yc_float_t, data) }, \
         } \
}


/**
 * @brief Pack a yc_float message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param data the float data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_yc_float_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_YC_FLOAT_LEN];
	_mav_put_float(buf, 0, data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_YC_FLOAT_LEN);
#else
	mavlink_yc_float_t packet;
	packet.data = data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_YC_FLOAT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_YC_FLOAT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_YC_FLOAT_LEN, MAVLINK_MSG_ID_YC_FLOAT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_YC_FLOAT_LEN);
#endif
}

/**
 * @brief Pack a yc_float message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param data the float data
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_yc_float_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_YC_FLOAT_LEN];
	_mav_put_float(buf, 0, data);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_YC_FLOAT_LEN);
#else
	mavlink_yc_float_t packet;
	packet.data = data;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_YC_FLOAT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_YC_FLOAT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_YC_FLOAT_LEN, MAVLINK_MSG_ID_YC_FLOAT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_YC_FLOAT_LEN);
#endif
}

/**
 * @brief Encode a yc_float struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param yc_float C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_yc_float_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_yc_float_t* yc_float)
{
	return mavlink_msg_yc_float_pack(system_id, component_id, msg, yc_float->data);
}

/**
 * @brief Encode a yc_float struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param yc_float C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_yc_float_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_yc_float_t* yc_float)
{
	return mavlink_msg_yc_float_pack_chan(system_id, component_id, chan, msg, yc_float->data);
}

/**
 * @brief Send a yc_float message
 * @param chan MAVLink channel to send the message
 *
 * @param data the float data
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_yc_float_send(mavlink_channel_t chan, float data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_YC_FLOAT_LEN];
	_mav_put_float(buf, 0, data);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YC_FLOAT, buf, MAVLINK_MSG_ID_YC_FLOAT_LEN, MAVLINK_MSG_ID_YC_FLOAT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YC_FLOAT, buf, MAVLINK_MSG_ID_YC_FLOAT_LEN);
#endif
#else
	mavlink_yc_float_t packet;
	packet.data = data;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YC_FLOAT, (const char *)&packet, MAVLINK_MSG_ID_YC_FLOAT_LEN, MAVLINK_MSG_ID_YC_FLOAT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_YC_FLOAT, (const char *)&packet, MAVLINK_MSG_ID_YC_FLOAT_LEN);
#endif
#endif
}

#endif

// MESSAGE YC_FLOAT UNPACKING


/**
 * @brief Get field data from yc_float message
 *
 * @return the float data
 */
static inline float mavlink_msg_yc_float_get_data(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a yc_float message into a struct
 *
 * @param msg The message to decode
 * @param yc_float C-struct to decode the message contents into
 */
static inline void mavlink_msg_yc_float_decode(const mavlink_message_t* msg, mavlink_yc_float_t* yc_float)
{
#if MAVLINK_NEED_BYTE_SWAP
	yc_float->data = mavlink_msg_yc_float_get_data(msg);
#else
	memcpy(yc_float, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_YC_FLOAT_LEN);
#endif
}
