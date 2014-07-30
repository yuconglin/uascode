// MESSAGE IF_COLLISION PACKING

#define MAVLINK_MSG_ID_IF_COLLISION 209

typedef struct __mavlink_if_collision_t
{
 int8_t if_collision; ///< flag indicating if collision is predicted
} mavlink_if_collision_t;

#define MAVLINK_MSG_ID_IF_COLLISION_LEN 1
#define MAVLINK_MSG_ID_209_LEN 1

#define MAVLINK_MSG_ID_IF_COLLISION_CRC 187
#define MAVLINK_MSG_ID_209_CRC 187



#define MAVLINK_MESSAGE_INFO_IF_COLLISION { \
	"IF_COLLISION", \
	1, \
	{  { "if_collision", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_if_collision_t, if_collision) }, \
         } \
}


/**
 * @brief Pack a if_collision message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param if_collision flag indicating if collision is predicted
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_if_collision_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int8_t if_collision)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IF_COLLISION_LEN];
	_mav_put_int8_t(buf, 0, if_collision);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IF_COLLISION_LEN);
#else
	mavlink_if_collision_t packet;
	packet.if_collision = if_collision;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IF_COLLISION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IF_COLLISION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IF_COLLISION_LEN, MAVLINK_MSG_ID_IF_COLLISION_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IF_COLLISION_LEN);
#endif
}

/**
 * @brief Pack a if_collision message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param if_collision flag indicating if collision is predicted
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_if_collision_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int8_t if_collision)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IF_COLLISION_LEN];
	_mav_put_int8_t(buf, 0, if_collision);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IF_COLLISION_LEN);
#else
	mavlink_if_collision_t packet;
	packet.if_collision = if_collision;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IF_COLLISION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_IF_COLLISION;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IF_COLLISION_LEN, MAVLINK_MSG_ID_IF_COLLISION_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IF_COLLISION_LEN);
#endif
}

/**
 * @brief Encode a if_collision struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param if_collision C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_if_collision_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_if_collision_t* if_collision)
{
	return mavlink_msg_if_collision_pack(system_id, component_id, msg, if_collision->if_collision);
}

/**
 * @brief Encode a if_collision struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param if_collision C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_if_collision_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_if_collision_t* if_collision)
{
	return mavlink_msg_if_collision_pack_chan(system_id, component_id, chan, msg, if_collision->if_collision);
}

/**
 * @brief Send a if_collision message
 * @param chan MAVLink channel to send the message
 *
 * @param if_collision flag indicating if collision is predicted
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_if_collision_send(mavlink_channel_t chan, int8_t if_collision)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_IF_COLLISION_LEN];
	_mav_put_int8_t(buf, 0, if_collision);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IF_COLLISION, buf, MAVLINK_MSG_ID_IF_COLLISION_LEN, MAVLINK_MSG_ID_IF_COLLISION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IF_COLLISION, buf, MAVLINK_MSG_ID_IF_COLLISION_LEN);
#endif
#else
	mavlink_if_collision_t packet;
	packet.if_collision = if_collision;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IF_COLLISION, (const char *)&packet, MAVLINK_MSG_ID_IF_COLLISION_LEN, MAVLINK_MSG_ID_IF_COLLISION_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IF_COLLISION, (const char *)&packet, MAVLINK_MSG_ID_IF_COLLISION_LEN);
#endif
#endif
}

#endif

// MESSAGE IF_COLLISION UNPACKING


/**
 * @brief Get field if_collision from if_collision message
 *
 * @return flag indicating if collision is predicted
 */
static inline int8_t mavlink_msg_if_collision_get_if_collision(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  0);
}

/**
 * @brief Decode a if_collision message into a struct
 *
 * @param msg The message to decode
 * @param if_collision C-struct to decode the message contents into
 */
static inline void mavlink_msg_if_collision_decode(const mavlink_message_t* msg, mavlink_if_collision_t* if_collision)
{
#if MAVLINK_NEED_BYTE_SWAP
	if_collision->if_collision = mavlink_msg_if_collision_get_if_collision(msg);
#else
	memcpy(if_collision, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_IF_COLLISION_LEN);
#endif
}
