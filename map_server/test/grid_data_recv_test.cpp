/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  Grid_data_recv_test.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2020/1/14
  * - Brief:     Node to test grid data receiving of global map
  ******************************************************************************
*/

#include "ros/ros.h"
#include <iostream>
#include "map_server/tcp_client.h"
#include "map_server/grid_data_pack_protocol.h"

void testPrintBuffer(std::vector<uint8_t>& buf){
	if(buf.size() > 0){
		for(auto element : buf){
			std::cout << static_cast<int>(element) << " ";
		}
		std::cout << std::endl;
	}
	else{
		ROS_WARN("grid_data_recv_test: buffer is empty!");
	}
}

unsigned char* gridDataUnpack(std::vector<uint8_t>& buf, int h_ind, int t_ind){
	unsigned char* grid_data = nullptr;

	int sec_bit = h_ind + 1;
	int w_bit = h_ind + 2;
	int h_bit = h_ind + 3;

	if(sec_bit < buf.size() && w_bit < buf.size() && h_bit < buf.size()){
		int map_w = static_cast<int>(buf[w_bit]);
		int map_h = static_cast<int>(buf[h_bit]);

		ROS_INFO_STREAM("grid_data_recv_test: bitset position " << h_ind
			<< " " << t_ind);
		
		if(t_ind - h_ind == map_w * map_h + 4){
			ROS_INFO("grid_data_recv_test: print recv grid info ");
			for(int c_ind = h_bit + 1; c_ind < t_ind; ++c_ind){
				std::cout << static_cast<int>(buf[c_ind]) << " ";
				grid_data = &buf[c_ind];
				++grid_data;
			}	
			std::cout << std::endl;
			return grid_data;	
		} 
	}
	else
		return grid_data;
}

void testGetGridData(std::vector<uint8_t>& buf){
	ROS_INFO("grid_data_recv_test: Receive buffer");
	// testPrintBuffer(buf);
	if(buf.size()){
		for(int ind = 0; ind < buf.size(); ++ind){
			if(static_cast<char>(buf[ind]) == GridDataPackProtocol::pack_head_){
				ROS_INFO("grid_data_recv_test: Found header");
				for(int b_ind = ind + 1; b_ind < buf.size(); ++b_ind){
					if(static_cast<char>(buf[b_ind]) == GridDataPackProtocol::pack_tail_){
						ROS_INFO("grid_data_recv_test: Found tail");
						gridDataUnpack(buf, ind, b_ind);
						break;
					}
				}
			}
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "grid_data_recv_test");
	ros::NodeHandle nh;

	std::vector<uint8_t> data_buf;

	// set tcp client
	TCPClient tcp_c("127.0.0.1", 6689, 125);
	tcp_c.setRecvProcess(boost::bind(&testGetGridData, _1));
	// tcp_c.setRecvProcess(boost::bind(&testPrintBuffer, _1));
	
	try{
		tcp_c.run();
	}
	catch(std::exception& e){
		ROS_ERROR("grid_data_recv_test", "Cannot start tcp client normally");
	}

	return 0;
}	
