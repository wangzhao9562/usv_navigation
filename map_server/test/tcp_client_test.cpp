/**
  ******************************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  tcp_client_test.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2020/1/2
  * - Brief:     Node to test tcp client 
********************************************************************************************
**/
#include "ros/ros.h"
#include "map_server/tcp_client.h"
#include "map_server/mavlink/v2.0/arms_usv_nav/mavlink.h"

void testMavUnpack(buffer_type& buf){
    if(buf.size()){
      for(int ind = 0; ind < buf.size(); ++ind){
        mavlink_message_t mav_msg;
        mavlink_status_t status;

        uint8_t c = buf[ind];
        /*try{
                c = tcp_buf_[ind];
        }
        catch(std::exception& e){
                ROS_ERROR_STREAM("tcp_client: error in buffer" << unsigned(c));
        }*/

        // ROS_INFO_STREAM("mav_link: parse char: " << unsigned(c));
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &mav_msg, &status)){
                ROS_INFO("mav_link: check message id");
                switch(mav_msg.msgid){
                        case MAVLINK_MSG_ID_MAP_INFO:
                        {
                                // get map name
                                uint8_t map_name[20];
                                ROS_INFO("mav_link: start unpack");
                                mavlink_msg_map_info_get_map_name(&mav_msg, map_name);
                                ROS_INFO("mav_link: get map name");
                                const char* map_name_c = reinterpret_cast<char*>(map_name);
                                std::string map_name_str = map_name_c;
                                std::cout << "map_name: " << map_name_str << std::endl;
                                // get map width 
                                uint32_t map_width;
                                map_width = mavlink_msg_map_info_get_map_width(&mav_msg);
                                ROS_INFO("mav_link: get map width");
                                std::cout << "map_width: " << map_width << std::endl;
                                // get map height
                                uint32_t map_height;
                                map_height = mavlink_msg_map_info_get_map_height(&mav_msg);
                                ROS_INFO("mav_link: get map height");
                                std::cout << "map_height: " << map_height << std::endl;
                                // get origin x
                                uint32_t origin_x;
                                origin_x = mavlink_msg_map_info_get_origin_x(&mav_msg);
                                ROS_INFO("mav_link: get origin x");
                                std::cout << "origin_x: " << origin_x << std::endl;
                                // get origin y
                                uint32_t origin_y;
                                origin_y = mavlink_msg_map_info_get_origin_y(&mav_msg);

                                ROS_INFO("mav_link: get origin y");
                                std::cout << "origin_y: " << origin_y << std::endl;
                                // get map x drift
                                uint32_t drift_x;
                                drift_x = mavlink_msg_map_info_get_x_in_last_map(&mav_msg);
                                ROS_INFO("mav_link: get drift x");
                                std::cout << "drift_x: " << drift_x << std::endl;
                                // get map y drift
                                uint32_t drift_y;
                                drift_y = mavlink_msg_map_info_get_y_in_last_map(&mav_msg);
                                ROS_INFO("mav_link: get drift y");
                                std::cout << "drift_y: " << drift_y << std::endl;
                                // get resolution
                                float resolution;
                                resolution = mavlink_msg_map_info_get_resolution(&mav_msg);
                                ROS_INFO("mav_link: get resolution");
                                std::cout << "resolution: " << resolution << std::endl;
                                ROS_INFO("tcp_client: unpacking finish");
                        }
                                break;
                        default:
                                break;
                }
        }
      }
    }
}


int main(int argc, char* argv[]){
	ros::init(argc, argv, "tcp_client_test");
	ros::NodeHandle nh;

	std::vector<uint8_t> data_buf;

	// TCPClient tcp_c("127.0.0.1", 16685, 256); // raw test
	TCPClient tcp_c("127.0.0.1", 16685, data_buf, 256); // mavlink test

	tcp_c.setRecvProcess(boost::bind(&testMavUnpack, data_buf)); // set callback handler

	try{
		tcp_c.run();
	}
	catch(std::exception& e){
		ROS_ERROR("tcp_client_test: Cannot start tcp client normally");	
	}
	
	return 0;
}

