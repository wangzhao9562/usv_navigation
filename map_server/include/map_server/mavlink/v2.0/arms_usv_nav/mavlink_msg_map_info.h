#pragma once
// MESSAGE MAP_INFO PACKING

#define MAVLINK_MSG_ID_MAP_INFO 0

MAVPACKED(
typedef struct __mavlink_map_info_t {
 uint32_t map_name_len; /*<  Length of map length*/
 uint32_t occupancy_grid[2073600]; /*<  Value of occupancy gird*/
 uint32_t map_width; /*<  Width of map*/
 uint32_t map_height; /*<  Height of map*/
 uint32_t origin_x; /*<  X coordinate of map origin*/
 uint32_t origin_y; /*<  Y coordinate of map origin*/
 uint32_t x_in_last_map; /*<  X coordinate of map in last map*/
 uint32_t y_in_last_map; /*<  Y cooridnate of map in last map*/
 float resolution; /*<  Resolution of map in meter*/
 uint8_t map_name[256]; /*<  Name of map*/
}) mavlink_map_info_t;

#define MAVLINK_MSG_ID_MAP_INFO_LEN 8294688
#define MAVLINK_MSG_ID_MAP_INFO_MIN_LEN 8294688
#define MAVLINK_MSG_ID_0_LEN 8294688
#define MAVLINK_MSG_ID_0_MIN_LEN 8294688

#define MAVLINK_MSG_ID_MAP_INFO_CRC 226
#define MAVLINK_MSG_ID_0_CRC 226

#define MAVLINK_MSG_MAP_INFO_FIELD_OCCUPANCY_GRID_LEN 2073600
#define MAVLINK_MSG_MAP_INFO_FIELD_MAP_NAME_LEN 256

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAP_INFO { \
    0, \
    "MAP_INFO", \
    10, \
    {  { "map_name", NULL, MAVLINK_TYPE_UINT8_T, 256, 8294432, offsetof(mavlink_map_info_t, map_name) }, \
         { "map_name_len", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_map_info_t, map_name_len) }, \
         { "occupancy_grid", NULL, MAVLINK_TYPE_UINT32_T, 2073600, 4, offsetof(mavlink_map_info_t, occupancy_grid) }, \
         { "map_width", NULL, MAVLINK_TYPE_UINT32_T, 0, 8294404, offsetof(mavlink_map_info_t, map_width) }, \
         { "map_height", NULL, MAVLINK_TYPE_UINT32_T, 0, 8294408, offsetof(mavlink_map_info_t, map_height) }, \
         { "origin_x", NULL, MAVLINK_TYPE_UINT32_T, 0, 8294412, offsetof(mavlink_map_info_t, origin_x) }, \
         { "origin_y", NULL, MAVLINK_TYPE_UINT32_T, 0, 8294416, offsetof(mavlink_map_info_t, origin_y) }, \
         { "x_in_last_map", NULL, MAVLINK_TYPE_UINT32_T, 0, 8294420, offsetof(mavlink_map_info_t, x_in_last_map) }, \
         { "y_in_last_map", NULL, MAVLINK_TYPE_UINT32_T, 0, 8294424, offsetof(mavlink_map_info_t, y_in_last_map) }, \
         { "resolution", NULL, MAVLINK_TYPE_FLOAT, 0, 8294428, offsetof(mavlink_map_info_t, resolution) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAP_INFO { \
    "MAP_INFO", \
    10, \
    {  { "map_name", NULL, MAVLINK_TYPE_UINT8_T, 256, 8294432, offsetof(mavlink_map_info_t, map_name) }, \
         { "map_name_len", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_map_info_t, map_name_len) }, \
         { "occupancy_grid", NULL, MAVLINK_TYPE_UINT32_T, 2073600, 4, offsetof(mavlink_map_info_t, occupancy_grid) }, \
         { "map_width", NULL, MAVLINK_TYPE_UINT32_T, 0, 8294404, offsetof(mavlink_map_info_t, map_width) }, \
         { "map_height", NULL, MAVLINK_TYPE_UINT32_T, 0, 8294408, offsetof(mavlink_map_info_t, map_height) }, \
         { "origin_x", NULL, MAVLINK_TYPE_UINT32_T, 0, 8294412, offsetof(mavlink_map_info_t, origin_x) }, \
         { "origin_y", NULL, MAVLINK_TYPE_UINT32_T, 0, 8294416, offsetof(mavlink_map_info_t, origin_y) }, \
         { "x_in_last_map", NULL, MAVLINK_TYPE_UINT32_T, 0, 8294420, offsetof(mavlink_map_info_t, x_in_last_map) }, \
         { "y_in_last_map", NULL, MAVLINK_TYPE_UINT32_T, 0, 8294424, offsetof(mavlink_map_info_t, y_in_last_map) }, \
         { "resolution", NULL, MAVLINK_TYPE_FLOAT, 0, 8294428, offsetof(mavlink_map_info_t, resolution) }, \
         } \
}
#endif

/**
 * @brief Pack a map_info message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param map_name  Name of map
 * @param map_name_len  Length of map length
 * @param occupancy_grid  Value of occupancy gird
 * @param map_width  Width of map
 * @param map_height  Height of map
 * @param origin_x  X coordinate of map origin
 * @param origin_y  Y coordinate of map origin
 * @param x_in_last_map  X coordinate of map in last map
 * @param y_in_last_map  Y cooridnate of map in last map
 * @param resolution  Resolution of map in meter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_map_info_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint8_t *map_name, uint32_t map_name_len, const uint32_t *occupancy_grid, uint32_t map_width, uint32_t map_height, uint32_t origin_x, uint32_t origin_y, uint32_t x_in_last_map, uint32_t y_in_last_map, float resolution)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAP_INFO_LEN];
    _mav_put_uint32_t(buf, 0, map_name_len);
    _mav_put_uint32_t(buf, 8294404, map_width);
    _mav_put_uint32_t(buf, 8294408, map_height);
    _mav_put_uint32_t(buf, 8294412, origin_x);
    _mav_put_uint32_t(buf, 8294416, origin_y);
    _mav_put_uint32_t(buf, 8294420, x_in_last_map);
    _mav_put_uint32_t(buf, 8294424, y_in_last_map);
    _mav_put_float(buf, 8294428, resolution);
    _mav_put_uint32_t_array(buf, 4, occupancy_grid, 2073600);
    _mav_put_uint8_t_array(buf, 8294432, map_name, 256);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAP_INFO_LEN);
#else
    mavlink_map_info_t packet;
    packet.map_name_len = map_name_len;
    packet.map_width = map_width;
    packet.map_height = map_height;
    packet.origin_x = origin_x;
    packet.origin_y = origin_y;
    packet.x_in_last_map = x_in_last_map;
    packet.y_in_last_map = y_in_last_map;
    packet.resolution = resolution;
    mav_array_memcpy(packet.occupancy_grid, occupancy_grid, sizeof(uint32_t)*2073600);
    mav_array_memcpy(packet.map_name, map_name, sizeof(uint8_t)*256);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAP_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAP_INFO;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAP_INFO_MIN_LEN, MAVLINK_MSG_ID_MAP_INFO_LEN, MAVLINK_MSG_ID_MAP_INFO_CRC);
}

/**
 * @brief Pack a map_info message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param map_name  Name of map
 * @param map_name_len  Length of map length
 * @param occupancy_grid  Value of occupancy gird
 * @param map_width  Width of map
 * @param map_height  Height of map
 * @param origin_x  X coordinate of map origin
 * @param origin_y  Y coordinate of map origin
 * @param x_in_last_map  X coordinate of map in last map
 * @param y_in_last_map  Y cooridnate of map in last map
 * @param resolution  Resolution of map in meter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_map_info_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint8_t *map_name,uint32_t map_name_len,const uint32_t *occupancy_grid,uint32_t map_width,uint32_t map_height,uint32_t origin_x,uint32_t origin_y,uint32_t x_in_last_map,uint32_t y_in_last_map,float resolution)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAP_INFO_LEN];
    _mav_put_uint32_t(buf, 0, map_name_len);
    _mav_put_uint32_t(buf, 8294404, map_width);
    _mav_put_uint32_t(buf, 8294408, map_height);
    _mav_put_uint32_t(buf, 8294412, origin_x);
    _mav_put_uint32_t(buf, 8294416, origin_y);
    _mav_put_uint32_t(buf, 8294420, x_in_last_map);
    _mav_put_uint32_t(buf, 8294424, y_in_last_map);
    _mav_put_float(buf, 8294428, resolution);
    _mav_put_uint32_t_array(buf, 4, occupancy_grid, 2073600);
    _mav_put_uint8_t_array(buf, 8294432, map_name, 256);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAP_INFO_LEN);
#else
    mavlink_map_info_t packet;
    packet.map_name_len = map_name_len;
    packet.map_width = map_width;
    packet.map_height = map_height;
    packet.origin_x = origin_x;
    packet.origin_y = origin_y;
    packet.x_in_last_map = x_in_last_map;
    packet.y_in_last_map = y_in_last_map;
    packet.resolution = resolution;
    mav_array_memcpy(packet.occupancy_grid, occupancy_grid, sizeof(uint32_t)*2073600);
    mav_array_memcpy(packet.map_name, map_name, sizeof(uint8_t)*256);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAP_INFO_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAP_INFO;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAP_INFO_MIN_LEN, MAVLINK_MSG_ID_MAP_INFO_LEN, MAVLINK_MSG_ID_MAP_INFO_CRC);
}

/**
 * @brief Encode a map_info struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param map_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_map_info_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_map_info_t* map_info)
{
    return mavlink_msg_map_info_pack(system_id, component_id, msg, map_info->map_name, map_info->map_name_len, map_info->occupancy_grid, map_info->map_width, map_info->map_height, map_info->origin_x, map_info->origin_y, map_info->x_in_last_map, map_info->y_in_last_map, map_info->resolution);
}

/**
 * @brief Encode a map_info struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param map_info C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_map_info_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_map_info_t* map_info)
{
    return mavlink_msg_map_info_pack_chan(system_id, component_id, chan, msg, map_info->map_name, map_info->map_name_len, map_info->occupancy_grid, map_info->map_width, map_info->map_height, map_info->origin_x, map_info->origin_y, map_info->x_in_last_map, map_info->y_in_last_map, map_info->resolution);
}

/**
 * @brief Send a map_info message
 * @param chan MAVLink channel to send the message
 *
 * @param map_name  Name of map
 * @param map_name_len  Length of map length
 * @param occupancy_grid  Value of occupancy gird
 * @param map_width  Width of map
 * @param map_height  Height of map
 * @param origin_x  X coordinate of map origin
 * @param origin_y  Y coordinate of map origin
 * @param x_in_last_map  X coordinate of map in last map
 * @param y_in_last_map  Y cooridnate of map in last map
 * @param resolution  Resolution of map in meter
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_map_info_send(mavlink_channel_t chan, const uint8_t *map_name, uint32_t map_name_len, const uint32_t *occupancy_grid, uint32_t map_width, uint32_t map_height, uint32_t origin_x, uint32_t origin_y, uint32_t x_in_last_map, uint32_t y_in_last_map, float resolution)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAP_INFO_LEN];
    _mav_put_uint32_t(buf, 0, map_name_len);
    _mav_put_uint32_t(buf, 8294404, map_width);
    _mav_put_uint32_t(buf, 8294408, map_height);
    _mav_put_uint32_t(buf, 8294412, origin_x);
    _mav_put_uint32_t(buf, 8294416, origin_y);
    _mav_put_uint32_t(buf, 8294420, x_in_last_map);
    _mav_put_uint32_t(buf, 8294424, y_in_last_map);
    _mav_put_float(buf, 8294428, resolution);
    _mav_put_uint32_t_array(buf, 4, occupancy_grid, 2073600);
    _mav_put_uint8_t_array(buf, 8294432, map_name, 256);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAP_INFO, buf, MAVLINK_MSG_ID_MAP_INFO_MIN_LEN, MAVLINK_MSG_ID_MAP_INFO_LEN, MAVLINK_MSG_ID_MAP_INFO_CRC);
#else
    mavlink_map_info_t packet;
    packet.map_name_len = map_name_len;
    packet.map_width = map_width;
    packet.map_height = map_height;
    packet.origin_x = origin_x;
    packet.origin_y = origin_y;
    packet.x_in_last_map = x_in_last_map;
    packet.y_in_last_map = y_in_last_map;
    packet.resolution = resolution;
    mav_array_memcpy(packet.occupancy_grid, occupancy_grid, sizeof(uint32_t)*2073600);
    mav_array_memcpy(packet.map_name, map_name, sizeof(uint8_t)*256);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAP_INFO, (const char *)&packet, MAVLINK_MSG_ID_MAP_INFO_MIN_LEN, MAVLINK_MSG_ID_MAP_INFO_LEN, MAVLINK_MSG_ID_MAP_INFO_CRC);
#endif
}

/**
 * @brief Send a map_info message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_map_info_send_struct(mavlink_channel_t chan, const mavlink_map_info_t* map_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_map_info_send(chan, map_info->map_name, map_info->map_name_len, map_info->occupancy_grid, map_info->map_width, map_info->map_height, map_info->origin_x, map_info->origin_y, map_info->x_in_last_map, map_info->y_in_last_map, map_info->resolution);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAP_INFO, (const char *)map_info, MAVLINK_MSG_ID_MAP_INFO_MIN_LEN, MAVLINK_MSG_ID_MAP_INFO_LEN, MAVLINK_MSG_ID_MAP_INFO_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAP_INFO_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_map_info_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *map_name, uint32_t map_name_len, const uint32_t *occupancy_grid, uint32_t map_width, uint32_t map_height, uint32_t origin_x, uint32_t origin_y, uint32_t x_in_last_map, uint32_t y_in_last_map, float resolution)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, map_name_len);
    _mav_put_uint32_t(buf, 8294404, map_width);
    _mav_put_uint32_t(buf, 8294408, map_height);
    _mav_put_uint32_t(buf, 8294412, origin_x);
    _mav_put_uint32_t(buf, 8294416, origin_y);
    _mav_put_uint32_t(buf, 8294420, x_in_last_map);
    _mav_put_uint32_t(buf, 8294424, y_in_last_map);
    _mav_put_float(buf, 8294428, resolution);
    _mav_put_uint32_t_array(buf, 4, occupancy_grid, 2073600);
    _mav_put_uint8_t_array(buf, 8294432, map_name, 256);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAP_INFO, buf, MAVLINK_MSG_ID_MAP_INFO_MIN_LEN, MAVLINK_MSG_ID_MAP_INFO_LEN, MAVLINK_MSG_ID_MAP_INFO_CRC);
#else
    mavlink_map_info_t *packet = (mavlink_map_info_t *)msgbuf;
    packet->map_name_len = map_name_len;
    packet->map_width = map_width;
    packet->map_height = map_height;
    packet->origin_x = origin_x;
    packet->origin_y = origin_y;
    packet->x_in_last_map = x_in_last_map;
    packet->y_in_last_map = y_in_last_map;
    packet->resolution = resolution;
    mav_array_memcpy(packet->occupancy_grid, occupancy_grid, sizeof(uint32_t)*2073600);
    mav_array_memcpy(packet->map_name, map_name, sizeof(uint8_t)*256);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAP_INFO, (const char *)packet, MAVLINK_MSG_ID_MAP_INFO_MIN_LEN, MAVLINK_MSG_ID_MAP_INFO_LEN, MAVLINK_MSG_ID_MAP_INFO_CRC);
#endif
}
#endif

#endif

// MESSAGE MAP_INFO UNPACKING


/**
 * @brief Get field map_name from map_info message
 *
 * @return  Name of map
 */
static inline uint16_t mavlink_msg_map_info_get_map_name(const mavlink_message_t* msg, uint8_t *map_name)
{
    return _MAV_RETURN_uint8_t_array(msg, map_name, 256,  8294432);
}

/**
 * @brief Get field map_name_len from map_info message
 *
 * @return  Length of map length
 */
static inline uint32_t mavlink_msg_map_info_get_map_name_len(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field occupancy_grid from map_info message
 *
 * @return  Value of occupancy gird
 */
static inline uint16_t mavlink_msg_map_info_get_occupancy_grid(const mavlink_message_t* msg, uint32_t *occupancy_grid)
{
    return _MAV_RETURN_uint32_t_array(msg, occupancy_grid, 2073600,  4);
}

/**
 * @brief Get field map_width from map_info message
 *
 * @return  Width of map
 */
static inline uint32_t mavlink_msg_map_info_get_map_width(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8294404);
}

/**
 * @brief Get field map_height from map_info message
 *
 * @return  Height of map
 */
static inline uint32_t mavlink_msg_map_info_get_map_height(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8294408);
}

/**
 * @brief Get field origin_x from map_info message
 *
 * @return  X coordinate of map origin
 */
static inline uint32_t mavlink_msg_map_info_get_origin_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8294412);
}

/**
 * @brief Get field origin_y from map_info message
 *
 * @return  Y coordinate of map origin
 */
static inline uint32_t mavlink_msg_map_info_get_origin_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8294416);
}

/**
 * @brief Get field x_in_last_map from map_info message
 *
 * @return  X coordinate of map in last map
 */
static inline uint32_t mavlink_msg_map_info_get_x_in_last_map(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8294420);
}

/**
 * @brief Get field y_in_last_map from map_info message
 *
 * @return  Y cooridnate of map in last map
 */
static inline uint32_t mavlink_msg_map_info_get_y_in_last_map(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8294424);
}

/**
 * @brief Get field resolution from map_info message
 *
 * @return  Resolution of map in meter
 */
static inline float mavlink_msg_map_info_get_resolution(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8294428);
}

/**
 * @brief Decode a map_info message into a struct
 *
 * @param msg The message to decode
 * @param map_info C-struct to decode the message contents into
 */
static inline void mavlink_msg_map_info_decode(const mavlink_message_t* msg, mavlink_map_info_t* map_info)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    map_info->map_name_len = mavlink_msg_map_info_get_map_name_len(msg);
    mavlink_msg_map_info_get_occupancy_grid(msg, map_info->occupancy_grid);
    map_info->map_width = mavlink_msg_map_info_get_map_width(msg);
    map_info->map_height = mavlink_msg_map_info_get_map_height(msg);
    map_info->origin_x = mavlink_msg_map_info_get_origin_x(msg);
    map_info->origin_y = mavlink_msg_map_info_get_origin_y(msg);
    map_info->x_in_last_map = mavlink_msg_map_info_get_x_in_last_map(msg);
    map_info->y_in_last_map = mavlink_msg_map_info_get_y_in_last_map(msg);
    map_info->resolution = mavlink_msg_map_info_get_resolution(msg);
    mavlink_msg_map_info_get_map_name(msg, map_info->map_name);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAP_INFO_LEN? msg->len : MAVLINK_MSG_ID_MAP_INFO_LEN;
        memset(map_info, 0, MAVLINK_MSG_ID_MAP_INFO_LEN);
    memcpy(map_info, _MAV_PAYLOAD(msg), len);
#endif
}
