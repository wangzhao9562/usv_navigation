/**
  ******************************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  tcp_server.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2020/1/6
  * - Brief:     Definition of TCPServer class to realize communication through TCP protocol
********************************************************************************************
**/

#ifndef TCP_SERVER_H_
#define TCP_SERVER_H_

#include <boost/bind/bind.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/chrono.hpp>
#include <boost/system/error_code.hpp>

/**
 * @brief TCP server used to send data stream to TCP client
 */
class TCPServer{
typedef boost::system::error_code error_code;
typedef boost::shared_lock<boost::shared_mutex> read_lock;
typedef boost::unique_lock<boost::shared_mutex> write_lock;

// Make brief type name
typedef TCPServer this_type;
typedef boost::asio::ip::tcp::acceptor acceptor_type;
typedef boost::asio::ip::tcp::endpoint endpoint_type;
typedef boost::asio::ip::tcp::socket socket_type;
typedef boost::shared_ptr<socket_type> sock_ptr;

public:
	/**
	 * @brief Constructor
	 * @param port TCP port number
	 */
	TCPServer(int port_num);

	/**
	 * @brief Deconstructor
	 */
	~TCPServer();

	/**
	 * @brief Run io_server
	 */
	void run(){
		tcp_io_.run();
	}

	/**
 	 * @brief Change write content
	 */
	void setWriteBuffer(uint8_t* buf, int buf_len);

private:
	/**
	 * @brief Start accept 
	 */
	void accept();

	/**
 	 * @brief Callback function of accept
	 */
	void acceptHandler(const error_code& ec, sock_ptr sock);

	/**
	 * @brief Callback function of data writing
	 */
	void writeHandler(const error_code& ec);

	/**
	 * @brief Used to print data in write buffer
	 */
	void testPrint();

private:
	boost::asio::io_service tcp_io_;
	socket_type *sock_t_; // socket 
	acceptor_type acceptor_;
	uint8_t* write_buf_; // buffer to write 
	size_t w_buf_len_; 

	boost::shared_mutex buf_mutex_;
};

#endif
