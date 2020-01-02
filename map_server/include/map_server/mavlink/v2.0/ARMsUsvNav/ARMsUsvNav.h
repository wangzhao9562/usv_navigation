/** @file
 *  @brief MAVLink comm protocol generated from ARMsUsvNav.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_ARMSUSVNAV_H
#define MAVLINK_ARMSUSVNAV_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_ARMSUSVNAV.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
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
#define MAVLINK_MESSAGE_CRCS {{0, 129, 4147488, 4147488, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_ARMSUSVNAV

// ENUM DEFINITIONS


/** @brief  */
#ifndef HAVE_ENUM_ARMS_USV_NAV_MAP_TRANSFER
#define HAVE_ENUM_ARMS_USV_NAV_MAP_TRANSFER
typedef enum ARMS_USV_NAV_MAP_TRANSFER
{
   INITIAL_MAP=0, /*  | */
   MAP_UPDATE=1, /*  | */
   MAP_TRANSFER_SUCCESS=9, /*  | */
   MAP_TRANSFER_FAILED=10, /*  | */
   ARMS_USV_NAV_MAP_TRANSFER_ENUM_END=11, /*  | */
} ARMS_USV_NAV_MAP_TRANSFER;
#endif

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
#endif // MAVLINK_ARMSUSVNAV_H
