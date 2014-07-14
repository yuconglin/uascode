// MESSAGE INTER_RECEIVE PACKING

#define MAVLINK_MSG_ID_INTER_RECEIVE 208

typedef struct __mavlink_inter_receive_t
{
 uint8_t receive; ///< if receive an intermediate waypoint
} mavlink_inter_receive_t;

#define MAVLINK_MSG_ID_INTER_RECEIVE_LEN 1
#define MAVLINK_MSG_ID_208_LEN 1

#define MAVLINK_MSG_ID_INTER_RECEIVE_CRC 233
#define MAVLINK_MSG_ID_208_CRC 233

#define MAVLINK_MESSAGE_INFO_INTER_RECEIVE { \
	"INTER_RECEIVE", \
	1, \
	{  { "receive", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_inter_receive_t, receive) }, \
         } \
}


/**
 * @brief Pack a inter_receive message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param receive if receive an intermediate waypoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_inter_receive_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t receive)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_INTER_RECEIVE_LEN];
	_mav_put_uint8_t(buf, 0, receive);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INTER_RECEIVE_LEN);
#else
	mavlink_inter_receive_t packet;
	packet.receive = receive;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INTER_RECEIVE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_INTER_RECEIVE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INTER_RECEIVE_LEN, MAVLINK_MSG_ID_INTER_RECEIVE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INTER_RECEIVE_LEN);
#endif
}

/**
 * @brief Pack a inter_receive message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param receive if receive an intermediate waypoint
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_inter_receive_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t receive)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_INTER_RECEIVE_LEN];
	_mav_put_uint8_t(buf, 0, receive);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INTER_RECEIVE_LEN);
#else
	mavlink_inter_receive_t packet;
	packet.receive = receive;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INTER_RECEIVE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_INTER_RECEIVE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INTER_RECEIVE_LEN, MAVLINK_MSG_ID_INTER_RECEIVE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INTER_RECEIVE_LEN);
#endif
}

/**
 * @brief Encode a inter_receive struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param inter_receive C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_inter_receive_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_inter_receive_t* inter_receive)
{
	return mavlink_msg_inter_receive_pack(system_id, component_id, msg, inter_receive->receive);
}

/**
 * @brief Encode a inter_receive struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param inter_receive C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_inter_receive_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_inter_receive_t* inter_receive)
{
	return mavlink_msg_inter_receive_pack_chan(system_id, component_id, chan, msg, inter_receive->receive);
}

/**
 * @brief Send a inter_receive message
 * @param chan MAVLink channel to send the message
 *
 * @param receive if receive an intermediate waypoint
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_inter_receive_send(mavlink_channel_t chan, uint8_t receive)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_INTER_RECEIVE_LEN];
	_mav_put_uint8_t(buf, 0, receive);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTER_RECEIVE, buf, MAVLINK_MSG_ID_INTER_RECEIVE_LEN, MAVLINK_MSG_ID_INTER_RECEIVE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTER_RECEIVE, buf, MAVLINK_MSG_ID_INTER_RECEIVE_LEN);
#endif
#else
	mavlink_inter_receive_t packet;
	packet.receive = receive;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTER_RECEIVE, (const char *)&packet, MAVLINK_MSG_ID_INTER_RECEIVE_LEN, MAVLINK_MSG_ID_INTER_RECEIVE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTER_RECEIVE, (const char *)&packet, MAVLINK_MSG_ID_INTER_RECEIVE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_INTER_RECEIVE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_inter_receive_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t receive)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, receive);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTER_RECEIVE, buf, MAVLINK_MSG_ID_INTER_RECEIVE_LEN, MAVLINK_MSG_ID_INTER_RECEIVE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTER_RECEIVE, buf, MAVLINK_MSG_ID_INTER_RECEIVE_LEN);
#endif
#else
	mavlink_inter_receive_t *packet = (mavlink_inter_receive_t *)msgbuf;
	packet->receive = receive;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTER_RECEIVE, (const char *)packet, MAVLINK_MSG_ID_INTER_RECEIVE_LEN, MAVLINK_MSG_ID_INTER_RECEIVE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INTER_RECEIVE, (const char *)packet, MAVLINK_MSG_ID_INTER_RECEIVE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE INTER_RECEIVE UNPACKING


/**
 * @brief Get field receive from inter_receive message
 *
 * @return if receive an intermediate waypoint
 */
static inline uint8_t mavlink_msg_inter_receive_get_receive(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a inter_receive message into a struct
 *
 * @param msg The message to decode
 * @param inter_receive C-struct to decode the message contents into
 */
static inline void mavlink_msg_inter_receive_decode(const mavlink_message_t* msg, mavlink_inter_receive_t* inter_receive)
{
#if MAVLINK_NEED_BYTE_SWAP
	inter_receive->receive = mavlink_msg_inter_receive_get_receive(msg);
#else
	memcpy(inter_receive, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_INTER_RECEIVE_LEN);
#endif
}
