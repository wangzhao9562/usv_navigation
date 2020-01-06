/** @file
 *  @brief MAVLink comm protocol generated from arms_usv_nav.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_ARMS_USV_NAV_H
#define MAVLINK_ARMS_USV_NAV_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_ARMS_USV_NAV.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 226, 8294688, 8294688, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_ARMS_USV_NAV

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_map_info.h"

// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_MAP_INFO}
# define MAVLINK_MESSAGE_NAMES {{ "MAP_INFO", 0 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_ARMS_USV_NAV_H
