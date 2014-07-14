// MESSAGE ADSB_MSG PACKING

#define MAVLINK_MSG_ID_ADSB_MSG 201

typedef struct __mavlink_adsb_msg_t
{
 int32_t address; ///< Participant Addrss
 float latitude; ///< Latitude
 float longitude; ///< Longitude
 int16_t altitude; ///< Altitude:actual= ddd*25-1000
 int16_t h_velocity; ///< Horizontal velocity in knots
 int16_t v_velocity; ///< Vertical velocity in 64fpm
 int8_t traffic_alert_status; ///< Traffic Alert Status.1:active
 int8_t address_type; ///< Address Type
 int8_t misc_indicator; ///< Miscellaneous Indicator
 int8_t NIC; ///< Navigation Integrity Category
 int8_t NACp; ///< Navigation Accuracy Category for Position
 int8_t heading; ///< Track/Heading in 360/128 degree,0=North,128=South
} mavlink_adsb_msg_t;

#define MAVLINK_MSG_ID_ADSB_MSG_LEN 24
#define MAVLINK_MSG_ID_201_LEN 24

#define MAVLINK_MSG_ID_ADSB_MSG_CRC 244
#define MAVLINK_MSG_ID_201_CRC 244



#define MAVLINK_MESSAGE_INFO_ADSB_MSG { \
	"ADSB_MSG", \
	12, \
	{  { "address", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_adsb_msg_t, address) }, \
         { "latitude", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_adsb_msg_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_adsb_msg_t, longitude) }, \
         { "altitude", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_adsb_msg_t, altitude) }, \
         { "h_velocity", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_adsb_msg_t, h_velocity) }, \
         { "v_velocity", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_adsb_msg_t, v_velocity) }, \
         { "traffic_alert_status", NULL, MAVLINK_TYPE_INT8_T, 0, 18, offsetof(mavlink_adsb_msg_t, traffic_alert_status) }, \
         { "address_type", NULL, MAVLINK_TYPE_INT8_T, 0, 19, offsetof(mavlink_adsb_msg_t, address_type) }, \
         { "misc_indicator", NULL, MAVLINK_TYPE_INT8_T, 0, 20, offsetof(mavlink_adsb_msg_t, misc_indicator) }, \
         { "NIC", NULL, MAVLINK_TYPE_INT8_T, 0, 21, offsetof(mavlink_adsb_msg_t, NIC) }, \
         { "NACp", NULL, MAVLINK_TYPE_INT8_T, 0, 22, offsetof(mavlink_adsb_msg_t, NACp) }, \
         { "heading", NULL, MAVLINK_TYPE_INT8_T, 0, 23, offsetof(mavlink_adsb_msg_t, heading) }, \
         } \
}


/**
 * @brief Pack a adsb_msg message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param traffic_alert_status Traffic Alert Status.1:active
 * @param address_type Address Type
 * @param address Participant Addrss
 * @param latitude Latitude
 * @param longitude Longitude
 * @param altitude Altitude:actual= ddd*25-1000
 * @param misc_indicator Miscellaneous Indicator
 * @param NIC Navigation Integrity Category
 * @param NACp Navigation Accuracy Category for Position
 * @param h_velocity Horizontal velocity in knots
 * @param v_velocity Vertical velocity in 64fpm
 * @param heading Track/Heading in 360/128 degree,0=North,128=South
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adsb_msg_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int8_t traffic_alert_status, int8_t address_type, int32_t address, float latitude, float longitude, int16_t altitude, int8_t misc_indicator, int8_t NIC, int8_t NACp, int16_t h_velocity, int16_t v_velocity, int8_t heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_MSG_LEN];
	_mav_put_int32_t(buf, 0, address);
	_mav_put_float(buf, 4, latitude);
	_mav_put_float(buf, 8, longitude);
	_mav_put_int16_t(buf, 12, altitude);
	_mav_put_int16_t(buf, 14, h_velocity);
	_mav_put_int16_t(buf, 16, v_velocity);
	_mav_put_int8_t(buf, 18, traffic_alert_status);
	_mav_put_int8_t(buf, 19, address_type);
	_mav_put_int8_t(buf, 20, misc_indicator);
	_mav_put_int8_t(buf, 21, NIC);
	_mav_put_int8_t(buf, 22, NACp);
	_mav_put_int8_t(buf, 23, heading);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADSB_MSG_LEN);
#else
	mavlink_adsb_msg_t packet;
	packet.address = address;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.h_velocity = h_velocity;
	packet.v_velocity = v_velocity;
	packet.traffic_alert_status = traffic_alert_status;
	packet.address_type = address_type;
	packet.misc_indicator = misc_indicator;
	packet.NIC = NIC;
	packet.NACp = NACp;
	packet.heading = heading;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADSB_MSG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADSB_MSG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADSB_MSG_LEN, MAVLINK_MSG_ID_ADSB_MSG_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ADSB_MSG_LEN);
#endif
}

/**
 * @brief Pack a adsb_msg message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param traffic_alert_status Traffic Alert Status.1:active
 * @param address_type Address Type
 * @param address Participant Addrss
 * @param latitude Latitude
 * @param longitude Longitude
 * @param altitude Altitude:actual= ddd*25-1000
 * @param misc_indicator Miscellaneous Indicator
 * @param NIC Navigation Integrity Category
 * @param NACp Navigation Accuracy Category for Position
 * @param h_velocity Horizontal velocity in knots
 * @param v_velocity Vertical velocity in 64fpm
 * @param heading Track/Heading in 360/128 degree,0=North,128=South
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_adsb_msg_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int8_t traffic_alert_status,int8_t address_type,int32_t address,float latitude,float longitude,int16_t altitude,int8_t misc_indicator,int8_t NIC,int8_t NACp,int16_t h_velocity,int16_t v_velocity,int8_t heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_MSG_LEN];
	_mav_put_int32_t(buf, 0, address);
	_mav_put_float(buf, 4, latitude);
	_mav_put_float(buf, 8, longitude);
	_mav_put_int16_t(buf, 12, altitude);
	_mav_put_int16_t(buf, 14, h_velocity);
	_mav_put_int16_t(buf, 16, v_velocity);
	_mav_put_int8_t(buf, 18, traffic_alert_status);
	_mav_put_int8_t(buf, 19, address_type);
	_mav_put_int8_t(buf, 20, misc_indicator);
	_mav_put_int8_t(buf, 21, NIC);
	_mav_put_int8_t(buf, 22, NACp);
	_mav_put_int8_t(buf, 23, heading);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ADSB_MSG_LEN);
#else
	mavlink_adsb_msg_t packet;
	packet.address = address;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.h_velocity = h_velocity;
	packet.v_velocity = v_velocity;
	packet.traffic_alert_status = traffic_alert_status;
	packet.address_type = address_type;
	packet.misc_indicator = misc_indicator;
	packet.NIC = NIC;
	packet.NACp = NACp;
	packet.heading = heading;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ADSB_MSG_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ADSB_MSG;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADSB_MSG_LEN, MAVLINK_MSG_ID_ADSB_MSG_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ADSB_MSG_LEN);
#endif
}

/**
 * @brief Encode a adsb_msg struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param adsb_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adsb_msg_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_adsb_msg_t* adsb_msg)
{
	return mavlink_msg_adsb_msg_pack(system_id, component_id, msg, adsb_msg->traffic_alert_status, adsb_msg->address_type, adsb_msg->address, adsb_msg->latitude, adsb_msg->longitude, adsb_msg->altitude, adsb_msg->misc_indicator, adsb_msg->NIC, adsb_msg->NACp, adsb_msg->h_velocity, adsb_msg->v_velocity, adsb_msg->heading);
}

/**
 * @brief Encode a adsb_msg struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param adsb_msg C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_adsb_msg_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_adsb_msg_t* adsb_msg)
{
	return mavlink_msg_adsb_msg_pack_chan(system_id, component_id, chan, msg, adsb_msg->traffic_alert_status, adsb_msg->address_type, adsb_msg->address, adsb_msg->latitude, adsb_msg->longitude, adsb_msg->altitude, adsb_msg->misc_indicator, adsb_msg->NIC, adsb_msg->NACp, adsb_msg->h_velocity, adsb_msg->v_velocity, adsb_msg->heading);
}

/**
 * @brief Send a adsb_msg message
 * @param chan MAVLink channel to send the message
 *
 * @param traffic_alert_status Traffic Alert Status.1:active
 * @param address_type Address Type
 * @param address Participant Addrss
 * @param latitude Latitude
 * @param longitude Longitude
 * @param altitude Altitude:actual= ddd*25-1000
 * @param misc_indicator Miscellaneous Indicator
 * @param NIC Navigation Integrity Category
 * @param NACp Navigation Accuracy Category for Position
 * @param h_velocity Horizontal velocity in knots
 * @param v_velocity Vertical velocity in 64fpm
 * @param heading Track/Heading in 360/128 degree,0=North,128=South
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_adsb_msg_send(mavlink_channel_t chan, int8_t traffic_alert_status, int8_t address_type, int32_t address, float latitude, float longitude, int16_t altitude, int8_t misc_indicator, int8_t NIC, int8_t NACp, int16_t h_velocity, int16_t v_velocity, int8_t heading)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ADSB_MSG_LEN];
	_mav_put_int32_t(buf, 0, address);
	_mav_put_float(buf, 4, latitude);
	_mav_put_float(buf, 8, longitude);
	_mav_put_int16_t(buf, 12, altitude);
	_mav_put_int16_t(buf, 14, h_velocity);
	_mav_put_int16_t(buf, 16, v_velocity);
	_mav_put_int8_t(buf, 18, traffic_alert_status);
	_mav_put_int8_t(buf, 19, address_type);
	_mav_put_int8_t(buf, 20, misc_indicator);
	_mav_put_int8_t(buf, 21, NIC);
	_mav_put_int8_t(buf, 22, NACp);
	_mav_put_int8_t(buf, 23, heading);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_MSG, buf, MAVLINK_MSG_ID_ADSB_MSG_LEN, MAVLINK_MSG_ID_ADSB_MSG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_MSG, buf, MAVLINK_MSG_ID_ADSB_MSG_LEN);
#endif
#else
	mavlink_adsb_msg_t packet;
	packet.address = address;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.altitude = altitude;
	packet.h_velocity = h_velocity;
	packet.v_velocity = v_velocity;
	packet.traffic_alert_status = traffic_alert_status;
	packet.address_type = address_type;
	packet.misc_indicator = misc_indicator;
	packet.NIC = NIC;
	packet.NACp = NACp;
	packet.heading = heading;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_MSG, (const char *)&packet, MAVLINK_MSG_ID_ADSB_MSG_LEN, MAVLINK_MSG_ID_ADSB_MSG_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ADSB_MSG, (const char *)&packet, MAVLINK_MSG_ID_ADSB_MSG_LEN);
#endif
#endif
}

#endif

// MESSAGE ADSB_MSG UNPACKING


/**
 * @brief Get field traffic_alert_status from adsb_msg message
 *
 * @return Traffic Alert Status.1:active
 */
static inline int8_t mavlink_msg_adsb_msg_get_traffic_alert_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  18);
}

/**
 * @brief Get field address_type from adsb_msg message
 *
 * @return Address Type
 */
static inline int8_t mavlink_msg_adsb_msg_get_address_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  19);
}

/**
 * @brief Get field address from adsb_msg message
 *
 * @return Participant Addrss
 */
static inline int32_t mavlink_msg_adsb_msg_get_address(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field latitude from adsb_msg message
 *
 * @return Latitude
 */
static inline float mavlink_msg_adsb_msg_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field longitude from adsb_msg message
 *
 * @return Longitude
 */
static inline float mavlink_msg_adsb_msg_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field altitude from adsb_msg message
 *
 * @return Altitude:actual= ddd*25-1000
 */
static inline int16_t mavlink_msg_adsb_msg_get_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field misc_indicator from adsb_msg message
 *
 * @return Miscellaneous Indicator
 */
static inline int8_t mavlink_msg_adsb_msg_get_misc_indicator(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  20);
}

/**
 * @brief Get field NIC from adsb_msg message
 *
 * @return Navigation Integrity Category
 */
static inline int8_t mavlink_msg_adsb_msg_get_NIC(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  21);
}

/**
 * @brief Get field NACp from adsb_msg message
 *
 * @return Navigation Accuracy Category for Position
 */
static inline int8_t mavlink_msg_adsb_msg_get_NACp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  22);
}

/**
 * @brief Get field h_velocity from adsb_msg message
 *
 * @return Horizontal velocity in knots
 */
static inline int16_t mavlink_msg_adsb_msg_get_h_velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field v_velocity from adsb_msg message
 *
 * @return Vertical velocity in 64fpm
 */
static inline int16_t mavlink_msg_adsb_msg_get_v_velocity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field heading from adsb_msg message
 *
 * @return Track/Heading in 360/128 degree,0=North,128=South
 */
static inline int8_t mavlink_msg_adsb_msg_get_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  23);
}

/**
 * @brief Decode a adsb_msg message into a struct
 *
 * @param msg The message to decode
 * @param adsb_msg C-struct to decode the message contents into
 */
static inline void mavlink_msg_adsb_msg_decode(const mavlink_message_t* msg, mavlink_adsb_msg_t* adsb_msg)
{
#if MAVLINK_NEED_BYTE_SWAP
	adsb_msg->address = mavlink_msg_adsb_msg_get_address(msg);
	adsb_msg->latitude = mavlink_msg_adsb_msg_get_latitude(msg);
	adsb_msg->longitude = mavlink_msg_adsb_msg_get_longitude(msg);
	adsb_msg->altitude = mavlink_msg_adsb_msg_get_altitude(msg);
	adsb_msg->h_velocity = mavlink_msg_adsb_msg_get_h_velocity(msg);
	adsb_msg->v_velocity = mavlink_msg_adsb_msg_get_v_velocity(msg);
	adsb_msg->traffic_alert_status = mavlink_msg_adsb_msg_get_traffic_alert_status(msg);
	adsb_msg->address_type = mavlink_msg_adsb_msg_get_address_type(msg);
	adsb_msg->misc_indicator = mavlink_msg_adsb_msg_get_misc_indicator(msg);
	adsb_msg->NIC = mavlink_msg_adsb_msg_get_NIC(msg);
	adsb_msg->NACp = mavlink_msg_adsb_msg_get_NACp(msg);
	adsb_msg->heading = mavlink_msg_adsb_msg_get_heading(msg);
#else
	memcpy(adsb_msg, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ADSB_MSG_LEN);
#endif
}
