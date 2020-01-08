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

int main(int argc, char* argv[]){
	ros::init(argc, argv, "tcp_client_test");
	ros::NodeHandle nh;

	TCPClient tcp_c("127.0.0.1", 16685, 256);
	
	// ros::Rate loop_rate(10);

	try{
		tcp_c.run();
	}
	catch(std::exception& e){
		ROS_ERROR("tcp_client_test: Cannot start tcp client normally");	
	}
	
	return 0;
}

