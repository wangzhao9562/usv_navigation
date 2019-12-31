// MESSAGE MAP_INFO support class

#pragma once

namespace mavlink {
namespace ARMsUsvNav {
namespace msg {

/**
 * @brief MAP_INFO message
 *
 * Int array stores the pixel value of map picture
 */
struct MAP_INFO : mavlink::Message {
    static constexpr msgid_t MSG_ID = 0;
    static constexpr size_t LENGTH = 4147228;
    static constexpr size_t MIN_LENGTH = 4147228;
    static constexpr uint8_t CRC_EXTRA = 193;
    static constexpr auto NAME = "MAP_INFO";


    std::array<uint16_t, 2073600> occupancy_grid; /*<  Value of occupancy gird */
    uint32_t map_width; /*<  Width of map */
    uint32_t map_height; /*<  Height of map */
    uint32_t origin_x; /*<  X coordinate of map origin */
    uint32_t origin_y; /*<  Y coordinate of map origin */
    uint32_t x_in_last_map; /*<  X coordinate of map in last map */
    uint32_t y_in_last_map; /*<  Y cooridnate of map in last map */
    float resolution; /*<  Resolution of map in meter */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  occupancy_grid: [" << to_string(occupancy_grid) << "]" << std::endl;
        ss << "  map_width: " << map_width << std::endl;
        ss << "  map_height: " << map_height << std::endl;
        ss << "  origin_x: " << origin_x << std::endl;
        ss << "  origin_y: " << origin_y << std::endl;
        ss << "  x_in_last_map: " << x_in_last_map << std::endl;
        ss << "  y_in_last_map: " << y_in_last_map << std::endl;
        ss << "  resolution: " << resolution << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << map_width;                     // offset: 0
        map << map_height;                    // offset: 4
        map << origin_x;                      // offset: 8
        map << origin_y;                      // offset: 12
        map << x_in_last_map;                 // offset: 16
        map << y_in_last_map;                 // offset: 20
        map << resolution;                    // offset: 24
        map << occupancy_grid;                // offset: 28
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> map_width;                     // offset: 0
        map >> map_height;                    // offset: 4
        map >> origin_x;                      // offset: 8
        map >> origin_y;                      // offset: 12
        map >> x_in_last_map;                 // offset: 16
        map >> y_in_last_map;                 // offset: 20
        map >> resolution;                    // offset: 24
        map >> occupancy_grid;                // offset: 28
    }
};

} // namespace msg
} // namespace ARMsUsvNav
} // namespace mavlink
