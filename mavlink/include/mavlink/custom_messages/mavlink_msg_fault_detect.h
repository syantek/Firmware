#pragma once
// MESSAGE FAULT_DETECT PACKING

#define MAVLINK_MSG_ID_FAULT_DETECT 227

MAVPACKED(
typedef struct __mavlink_fault_detect_t {
 float residual_power; /*< Residual power value*/
 float detector_threshold; /*< fault detection threshold parameter*/
}) mavlink_fault_detect_t;

#define MAVLINK_MSG_ID_FAULT_DETECT_LEN 8
#define MAVLINK_MSG_ID_FAULT_DETECT_MIN_LEN 8
#define MAVLINK_MSG_ID_227_LEN 8
#define MAVLINK_MSG_ID_227_MIN_LEN 8

#define MAVLINK_MSG_ID_FAULT_DETECT_CRC 218
#define MAVLINK_MSG_ID_227_CRC 218



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_FAULT_DETECT { \
    227, \
    "FAULT_DETECT", \
    2, \
    {  { "residual_power", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_fault_detect_t, residual_power) }, \
         { "detector_threshold", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_fault_detect_t, detector_threshold) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_FAULT_DETECT { \
    "FAULT_DETECT", \
    2, \
    {  { "residual_power", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_fault_detect_t, residual_power) }, \
         { "detector_threshold", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_fault_detect_t, detector_threshold) }, \
         } \
}
#endif

/**
 * @brief Pack a fault_detect message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param residual_power Residual power value
 * @param detector_threshold fault detection threshold parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fault_detect_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float residual_power, float detector_threshold)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FAULT_DETECT_LEN];
    _mav_put_float(buf, 0, residual_power);
    _mav_put_float(buf, 4, detector_threshold);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FAULT_DETECT_LEN);
#else
    mavlink_fault_detect_t packet;
    packet.residual_power = residual_power;
    packet.detector_threshold = detector_threshold;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FAULT_DETECT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FAULT_DETECT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FAULT_DETECT_MIN_LEN, MAVLINK_MSG_ID_FAULT_DETECT_LEN, MAVLINK_MSG_ID_FAULT_DETECT_CRC);
}

/**
 * @brief Pack a fault_detect message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param residual_power Residual power value
 * @param detector_threshold fault detection threshold parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_fault_detect_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float residual_power,float detector_threshold)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FAULT_DETECT_LEN];
    _mav_put_float(buf, 0, residual_power);
    _mav_put_float(buf, 4, detector_threshold);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FAULT_DETECT_LEN);
#else
    mavlink_fault_detect_t packet;
    packet.residual_power = residual_power;
    packet.detector_threshold = detector_threshold;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FAULT_DETECT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_FAULT_DETECT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FAULT_DETECT_MIN_LEN, MAVLINK_MSG_ID_FAULT_DETECT_LEN, MAVLINK_MSG_ID_FAULT_DETECT_CRC);
}

/**
 * @brief Encode a fault_detect struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param fault_detect C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fault_detect_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_fault_detect_t* fault_detect)
{
    return mavlink_msg_fault_detect_pack(system_id, component_id, msg, fault_detect->residual_power, fault_detect->detector_threshold);
}

/**
 * @brief Encode a fault_detect struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fault_detect C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_fault_detect_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_fault_detect_t* fault_detect)
{
    return mavlink_msg_fault_detect_pack_chan(system_id, component_id, chan, msg, fault_detect->residual_power, fault_detect->detector_threshold);
}

/**
 * @brief Send a fault_detect message
 * @param chan MAVLink channel to send the message
 *
 * @param residual_power Residual power value
 * @param detector_threshold fault detection threshold parameter
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_fault_detect_send(mavlink_channel_t chan, float residual_power, float detector_threshold)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_FAULT_DETECT_LEN];
    _mav_put_float(buf, 0, residual_power);
    _mav_put_float(buf, 4, detector_threshold);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FAULT_DETECT, buf, MAVLINK_MSG_ID_FAULT_DETECT_MIN_LEN, MAVLINK_MSG_ID_FAULT_DETECT_LEN, MAVLINK_MSG_ID_FAULT_DETECT_CRC);
#else
    mavlink_fault_detect_t packet;
    packet.residual_power = residual_power;
    packet.detector_threshold = detector_threshold;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FAULT_DETECT, (const char *)&packet, MAVLINK_MSG_ID_FAULT_DETECT_MIN_LEN, MAVLINK_MSG_ID_FAULT_DETECT_LEN, MAVLINK_MSG_ID_FAULT_DETECT_CRC);
#endif
}

/**
 * @brief Send a fault_detect message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_fault_detect_send_struct(mavlink_channel_t chan, const mavlink_fault_detect_t* fault_detect)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_fault_detect_send(chan, fault_detect->residual_power, fault_detect->detector_threshold);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FAULT_DETECT, (const char *)fault_detect, MAVLINK_MSG_ID_FAULT_DETECT_MIN_LEN, MAVLINK_MSG_ID_FAULT_DETECT_LEN, MAVLINK_MSG_ID_FAULT_DETECT_CRC);
#endif
}

#if MAVLINK_MSG_ID_FAULT_DETECT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_fault_detect_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float residual_power, float detector_threshold)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, residual_power);
    _mav_put_float(buf, 4, detector_threshold);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FAULT_DETECT, buf, MAVLINK_MSG_ID_FAULT_DETECT_MIN_LEN, MAVLINK_MSG_ID_FAULT_DETECT_LEN, MAVLINK_MSG_ID_FAULT_DETECT_CRC);
#else
    mavlink_fault_detect_t *packet = (mavlink_fault_detect_t *)msgbuf;
    packet->residual_power = residual_power;
    packet->detector_threshold = detector_threshold;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FAULT_DETECT, (const char *)packet, MAVLINK_MSG_ID_FAULT_DETECT_MIN_LEN, MAVLINK_MSG_ID_FAULT_DETECT_LEN, MAVLINK_MSG_ID_FAULT_DETECT_CRC);
#endif
}
#endif

#endif

// MESSAGE FAULT_DETECT UNPACKING


/**
 * @brief Get field residual_power from fault_detect message
 *
 * @return Residual power value
 */
static inline float mavlink_msg_fault_detect_get_residual_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field detector_threshold from fault_detect message
 *
 * @return fault detection threshold parameter
 */
static inline float mavlink_msg_fault_detect_get_detector_threshold(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a fault_detect message into a struct
 *
 * @param msg The message to decode
 * @param fault_detect C-struct to decode the message contents into
 */
static inline void mavlink_msg_fault_detect_decode(const mavlink_message_t* msg, mavlink_fault_detect_t* fault_detect)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    fault_detect->residual_power = mavlink_msg_fault_detect_get_residual_power(msg);
    fault_detect->detector_threshold = mavlink_msg_fault_detect_get_detector_threshold(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_FAULT_DETECT_LEN? msg->len : MAVLINK_MSG_ID_FAULT_DETECT_LEN;
        memset(fault_detect, 0, MAVLINK_MSG_ID_FAULT_DETECT_LEN);
    memcpy(fault_detect, _MAV_PAYLOAD(msg), len);
#endif
}
