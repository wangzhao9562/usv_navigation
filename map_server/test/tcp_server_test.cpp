/**
  ******************************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  tcp_server_test.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2020/1/2
  * - Brief:     Node to test tcp server
********************************************************************************************
**/
#include "ros/ros.h"
#include "map_server/tcp_server.h"
#include "map_server/mavlink/v2.0/arms_usv_nav/mavlink.h"

int main(int argc, char* argv[]){
	ros::init(argc, argv, "tcp_server_test");
	ros::NodeHandle nh;
	
	ROS_INFO("tcp_server_test: prepare to create server");
	TCPServer tcp_s(6688);	

	mavlink_message_t mav_msg;
	uint8_t  map_info[256];
	memset(map_info, 0, 256);
	
	const char* map_name_c = "test pic";
	uint8_t* map_name = reinterpret_cast<uint8_t*>(const_cast<char*>(map_name_c));
	uint32_t map_name_len = strlen(map_name_c) + 1;
	uint32_t map_width = 10;
	uint32_t map_height = 10;
	uint32_t origin_x = 5;
	uint32_t origin_y = 5;
	uint32_t x_in_last_map = 0;
	uint32_t y_in_last_map = 0;
	float resolution = 0.05;

	// pack mavlink message	
	try{
		ROS_INFO("tcp_server_test: prepare mavlink message packing");
		mavlink_msg_map_info_pack(1, 2, &mav_msg, map_name, map_name_len, map_width, map_height, origin_x, origin_y, x_in_last_map, y_in_last_map, resolution);
		ROS_INFO("tcp_server_test: prepare transform mavlink message to buffer");
		mavlink_msg_to_send_buffer(map_info, &mav_msg);
	}
	catch(std::exception& e)
	{
		ROS_ERROR("tcp_server_test: error in mavlink msg packing");
	}

	try{
		ROS_INFO("tcp_server_test: set buffer");
		tcp_s.setWriteBuffer(map_info, sizeof(map_info) / sizeof(uint8_t));

		ROS_INFO("tcp_server_test: Finish buffer setting");
	}
	catch(std::exception& e){
		ROS_ERROR("tcp_server_test: Error to set write buffer!");
	}

	// ros::Rate loop_rate(10);

	try{
		tcp_s.run();
	}
	catch(std::exception& e){
		ROS_ERROR("tcp_server_test: Error to start tcp server!");
	}

	return 0;
}
