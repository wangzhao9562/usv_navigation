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
#include "map_server/tcp_client.h"

TCPClient::TCPClient(std::string ip, size_t port, size_t buf_len) : tcp_buf_(buf_len, 0), 
    tcp_ep_(address_type::from_string(ip), port), sock_t_(NULL), is_listen_(true)
{
    // Initialize components
    initialize();
}

TCPClient::~TCPClient(){
    if(sock_t_){
        delete sock_t_;
    }
    sock_t_ = NULL;
}

void TCPClient::stopListen(){
    {
        write_lock set_status(listen_mutex_); // writting lock
        is_listen_ = false;
    }
}

void TCPClient::getBuf(buffer_type& buf){
    {
        read_lock read_buf(buf_mutex_); // reading buffer
        buf.clear();
        buf = tcp_buf_;
    }
}

void TCPClient::initialize()
{
    sock_t_ = new socket_type(tcp_io_);
    sock_ptr sock(sock_t_); // create socket pointer

    boost::thread read_thread(boost::bind(&this_type::recvData, this, sock)); // start read thread   
}


void TCPClient::recvData(sock_ptr sock){
    {
        read_lock listen(listen_mutex_); // reading lock
        while(is_listen_){
            sock->async_connect(tcp_ep_, boost::bind(&this_type::connHandler, 
                                                        this, boost::asio::placeholders::error, sock));
            boost::this_thread::sleep(boost::posix_time::millisec(200)); // thread sleep
        }
    }
}

void TCPClient::connHandler(const error_code& ec, sock_ptr sock){
    if(ec){
        return;
    }
    // read data from port
    {
        write_lock write_buf(buf_mutex_); // writting buffer
        sock->async_read_some(boost::asio::buffer(tcp_buf_), 
                        boost::bind(&this_type::readHandler, this, boost::asio::placeholders::error));
    }
}

void TCPClient::readHandler(const error_code& ec){
    if(ec){
        return;
    }
    // do nothing
    // testPrint();
}

void TCPClient::testPrint(){
    // print data
    std::cout << &tcp_buf_[0] << std::endl;
}
