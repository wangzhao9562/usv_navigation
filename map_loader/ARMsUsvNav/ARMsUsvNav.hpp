/** @file
 *	@brief MAVLink comm protocol generated from ARMsUsvNav.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace ARMsUsvNav {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (trought @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 1> MESSAGE_ENTRIES {{ {0, 193, 4147228, 4147228, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 2;


// ENUM DEFINITIONS


/** @brief  */
enum class ARMS_USV_NAV_MAP_TRANSFER
{
    INITIAL_MAP=0, /*  | */
    MAP_UPDATE=1, /*  | */
    MAP_TRANSFER_SUCCESS=9, /*  | */
    MAP_TRANSFER_FAILED=10, /*  | */
};

//! ARMS_USV_NAV_MAP_TRANSFER ENUM_END
constexpr auto ARMS_USV_NAV_MAP_TRANSFER_ENUM_END = 11;


} // namespace ARMsUsvNav
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_map_info.hpp"

// base include

