// MESSAGE WP_NUMBER PACKING

#define MAVLINK_MSG_ID_WP_NUMBER 210

typedef struct __mavlink_wp_number_t
{
 uint8_t number; ///< Number of Waypoints
} mavlink_wp_number_t;

#define MAVLINK_MSG_ID_WP_NUMBER_LEN 1
#define MAVLINK_MSG_ID_210_LEN 1

#define MAVLINK_MSG_ID_WP_NUMBER_CRC 69
#define MAVLINK_MSG_ID_210_CRC 69



#define MAVLINK_MESSAGE_INFO_WP_NUMBER { \
	"WP_NUMBER", \
	1, \
	{  { "number", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_wp_number_t, number) }, \
         } \
}


/**
 * @brief Pack a wp_number message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param number Number of Waypoints
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wp_number_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WP_NUMBER_LEN];
	_mav_put_uint8_t(buf, 0, number);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WP_NUMBER_LEN);
#else
	mavlink_wp_number_t packet;
	packet.number = number;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WP_NUMBER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_WP_NUMBER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WP_NUMBER_LEN, MAVLINK_MSG_ID_WP_NUMBER_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WP_NUMBER_LEN);
#endif
}

/**
 * @brief Pack a wp_number message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param number Number of Waypoints
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wp_number_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WP_NUMBER_LEN];
	_mav_put_uint8_t(buf, 0, number);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WP_NUMBER_LEN);
#else
	mavlink_wp_number_t packet;
	packet.number = number;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WP_NUMBER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_WP_NUMBER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WP_NUMBER_LEN, MAVLINK_MSG_ID_WP_NUMBER_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WP_NUMBER_LEN);
#endif
}

/**
 * @brief Encode a wp_number struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wp_number C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wp_number_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wp_number_t* wp_number)
{
	return mavlink_msg_wp_number_pack(system_id, component_id, msg, wp_number->number);
}

/**
 * @brief Encode a wp_number struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wp_number C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wp_number_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_wp_number_t* wp_number)
{
	return mavlink_msg_wp_number_pack_chan(system_id, component_id, chan, msg, wp_number->number);
}

/**
 * @brief Send a wp_number message
 * @param chan MAVLink channel to send the message
 *
 * @param number Number of Waypoints
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wp_number_send(mavlink_channel_t chan, uint8_t number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WP_NUMBER_LEN];
	_mav_put_uint8_t(buf, 0, number);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WP_NUMBER, buf, MAVLINK_MSG_ID_WP_NUMBER_LEN, MAVLINK_MSG_ID_WP_NUMBER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WP_NUMBER, buf, MAVLINK_MSG_ID_WP_NUMBER_LEN);
#endif
#else
	mavlink_wp_number_t packet;
	packet.number = number;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WP_NUMBER, (const char *)&packet, MAVLINK_MSG_ID_WP_NUMBER_LEN, MAVLINK_MSG_ID_WP_NUMBER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WP_NUMBER, (const char *)&packet, MAVLINK_MSG_ID_WP_NUMBER_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_WP_NUMBER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_wp_number_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t number)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, number);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WP_NUMBER, buf, MAVLINK_MSG_ID_WP_NUMBER_LEN, MAVLINK_MSG_ID_WP_NUMBER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WP_NUMBER, buf, MAVLINK_MSG_ID_WP_NUMBER_LEN);
#endif
#else
	mavlink_wp_number_t *packet = (mavlink_wp_number_t *)msgbuf;
	packet->number = number;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WP_NUMBER, (const char *)packet, MAVLINK_MSG_ID_WP_NUMBER_LEN, MAVLINK_MSG_ID_WP_NUMBER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WP_NUMBER, (const char *)packet, MAVLINK_MSG_ID_WP_NUMBER_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE WP_NUMBER UNPACKING


/**
 * @brief Get field number from wp_number message
 *
 * @return Number of Waypoints
 */
static inline uint8_t mavlink_msg_wp_number_get_number(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a wp_number message into a struct
 *
 * @param msg The message to decode
 * @param wp_number C-struct to decode the message contents into
 */
static inline void mavlink_msg_wp_number_decode(const mavlink_message_t* msg, mavlink_wp_number_t* wp_number)
{
#if MAVLINK_NEED_BYTE_SWAP
	wp_number->number = mavlink_msg_wp_number_get_number(msg);
#else
	memcpy(wp_number, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_WP_NUMBER_LEN);
#endif
}
