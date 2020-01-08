/**
  ******************************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  tcp_client.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2020/1/2
  * - Brief:     Implementation of TCPClient class to realize communication through TCP protocol
********************************************************************************************
**/
#include <iostream>
#include <string>
#include "map_server/tcp_client.h"
#include "map_server/mavlink/v2.0/arms_usv_nav/mavlink.h"
#include "ros/ros.h"

TCPClient::TCPClient(std::string ip, size_t port, size_t buf_size) : tcp_buf_(buf_size, 0), tcp_ep_(address_type::from_string(ip), port), sock_t_(nullptr), is_listen_(true)
{
    // Initialize components
    connect();
}

TCPClient::TCPClient(std::string ip, size_t port, buffer_type& buf, size_t buf_size) : tcp_buf_(buf_size, 0), tcp_ep_(address_type::from_string(ip), port), sock_t_(nullptr), is_listen_(true), recv_proc_(boost::bind(&this_type::testPrint, this, _1)){
    	
    connect(buf);
}

TCPClient::~TCPClient(){
    // if(sock_t_ != nullptr){
    //     delete sock_t_;
    // sock_t_ = nullptr;
    // }
}

void TCPClient::stopListen(){
    {
        write_lock set_status(listen_mutex_); // writting lock
        is_listen_ = false;
    }
}


void TCPClient::setRecvProcess(recv_proc_type recv_proc){
    recv_proc_ = recv_proc;
}


void TCPClient::getBuf(buffer_type& buf){
    {
        read_lock read_buf(buf_mutex_); // reading buffer
        buf.clear();
        buf = tcp_buf_;
    }
}

void TCPClient::connect()
{
    // sock_t_ = new socket_type(tcp_io_);
    // sock_ptr sock(sock_t_); // create socket pointer
    sock_ptr sock(new socket_type(tcp_io_));
    // boost::thread read_thread(boost::bind(&this_type::recvData, this, sock)); // start read thread   
    sock->async_connect(tcp_ep_, boost::bind(&this_type::connHandler, this, boost::asio::placeholders::error, sock));
}

void TCPClient::connect(buffer_type& buf)
{
    sock_ptr sock(new socket_type(tcp_io_));
    
    sock->async_connect(tcp_ep_, boost::bind(&this_type::connHandler, this, buf, boost::asio::placeholders::error, sock));
}

void TCPClient::recvData(sock_ptr sock){
    while(is_listen_){
      sock->async_connect(tcp_ep_, boost::bind(&this_type::connHandler, 
                                                        this, boost::asio::placeholders::error, sock));
      boost::this_thread::sleep(boost::posix_time::millisec(200)); // thread sleep
    }
}

void TCPClient::connHandler(const error_code& ec, sock_ptr sock){
    if(ec){
	ROS_WARN("tcp_client: error in connect");
        return;
    }
    // read data from port
    {
        write_lock write_buf(buf_mutex_); // writting buffer
        sock->async_read_some(boost::asio::buffer(tcp_buf_), boost::bind(&this_type::readHandler, this, boost::asio::placeholders::error));
        // sock->async_read_some(boost::asio::buffer(tcp_buf_), boost::bind(&r_handler_, this, boost::asio::placeholders::error));
    }
    if(ros::ok()){
	connect();
    }
    else{
        return;
    }	
}

void TCPClient::connHandler(buffer_type& buf, const error_code& ec, sock_ptr sock){
    if(ec){
	ROS_WARN("tcp_client: error in connect");
        return;
    }
    // read data from port
    {
        write_lock write_buf(buf_mutex_); // writting buffer
        sock->async_read_some(boost::asio::buffer(buf), boost::bind(&this_type::readHandler, this, boost::asio::placeholders::error));
        // sock->async_read_some(boost::asio::buffer(tcp_buf_), boost::bind(&r_handler_, this, boost::asio::placeholders::error));
    }
    if(ros::ok()){
	connect();
    }
    else{
        return;
    }	
}

void TCPClient::readHandler(const error_code& ec){
    if(ec){
        return;
    }
    // testPrint(tcp_buf_); 
    testMavUnpack(tcp_buf_);
}

void TCPClient::testPrint(buffer_type& buf){
    // print data
    for(auto x : buf){
      std::cout << unsigned(x) << " ";
    }
    std::cout << std::endl;
}

void TCPClient::testMavUnpack(buffer_type& buf){
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
