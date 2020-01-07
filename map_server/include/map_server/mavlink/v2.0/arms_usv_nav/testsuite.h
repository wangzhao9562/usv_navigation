/** @file
 *    @brief MAVLink comm protocol testsuite generated from arms_usv_nav.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef ARMS_USV_NAV_TESTSUITE_H
#define ARMS_USV_NAV_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_arms_usv_nav(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_arms_usv_nav(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_map_info(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MAP_INFO >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_map_info_t packet_in = {
        963497464,963497672,963497880,963498088,963498296,963498504,963498712,213.0,{ 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120 }
    };
    mavlink_map_info_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.map_name_len = packet_in.map_name_len;
        packet1.map_width = packet_in.map_width;
        packet1.map_height = packet_in.map_height;
        packet1.origin_x = packet_in.origin_x;
        packet1.origin_y = packet_in.origin_y;
        packet1.x_in_last_map = packet_in.x_in_last_map;
        packet1.y_in_last_map = packet_in.y_in_last_map;
        packet1.resolution = packet_in.resolution;
        
        mav_array_memcpy(packet1.map_name, packet_in.map_name, sizeof(uint8_t)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MAP_INFO_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MAP_INFO_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_map_info_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_map_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_map_info_pack(system_id, component_id, &msg , packet1.map_name , packet1.map_name_len , packet1.map_width , packet1.map_height , packet1.origin_x , packet1.origin_y , packet1.x_in_last_map , packet1.y_in_last_map , packet1.resolution );
    mavlink_msg_map_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_map_info_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.map_name , packet1.map_name_len , packet1.map_width , packet1.map_height , packet1.origin_x , packet1.origin_y , packet1.x_in_last_map , packet1.y_in_last_map , packet1.resolution );
    mavlink_msg_map_info_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_map_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_map_info_send(MAVLINK_COMM_1 , packet1.map_name , packet1.map_name_len , packet1.map_width , packet1.map_height , packet1.origin_x , packet1.origin_y , packet1.x_in_last_map , packet1.y_in_last_map , packet1.resolution );
    mavlink_msg_map_info_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_arms_usv_nav(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_map_info(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ARMS_USV_NAV_TESTSUITE_H
