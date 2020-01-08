/**
  ******************************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  tcp_client.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2020/1/2
  * - Brief:     Definition of TCPClient class to realize communication through TCP protocol
********************************************************************************************
**/

#ifndef TCP_CLIENT_H_
#define TCP_CLIENT_H_

#include <vector>
#include <string>
#include <boost/bind/bind.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/thread.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/chrono.hpp>
#include <boost/system/error_code.hpp>
#include <boost/function.hpp>

// typedef boost::system::error_code error_code;
// typedef boost::shared_lock<boost::shared_mutex> read_lock;
// typedef boost::unique_lock<boost::shared_mutex> write_lock;

/**
 * @brief TCP client used to receive data stream from TCP server
 */
class TCPClient{
typedef boost::system::error_code error_code;
typedef boost::shared_lock<boost::shared_mutex> read_lock;
typedef boost::unique_lock<boost::shared_mutex> write_lock;

// Make brief type name
typedef TCPClient this_type;
typedef boost::asio::ip::tcp::endpoint endpoint_type;
typedef boost::asio::ip::address address_type;
typedef boost::asio::ip::tcp::socket socket_type;
typedef boost::shared_ptr<socket_type> sock_ptr;
typedef std::vector<uint8_t> buffer_type;

typedef boost::function<void(buffer_type&)> recv_proc_type;

public:
    /**
     * @brief Constructor
     * @param ip TCP ip address
     * @param port TCP port number
     */
    TCPClient(std::string ip, size_t port, size_t buf_size = 2048);

    /**
     * @brief Constructor
     * @param ip TCP ip address
     * @param port TCP port number
     */
    TCPClient(std::string ip, size_t port, buffer_type& buf, size_t buf_size = 2048);
    
    /**
     * @brief Deconstructor
     */
    ~TCPClient();

    /**
     * @brief Run io_server
     */
    void run(){
        tcp_io_.run();
    }

    /**
     * @brief Set data receive processor
     * @param Process functor
     */
    void setRecvProcess(recv_proc_type recv_proc);

    /**
     * @brief Stop listen to the port and receive data
     */
    void stopListen();

    /**
     * @brief Get buffer data
     * @param buf Outside buffer var to store data
     * @param buf_len Length of data in buffer
     */
    void getBuf(uint8_t buf[], size_t& buf_len){};

    /**
     * @brief Get buffer data
     * @param buf Outside buffer var in form of vector to store data
     */
    void getBuf(buffer_type& buf);

private:
    /**
     * @brief Initialize TCP components, such as sockets
     */
    void connect();
    
    /**
     * @brief Initialize TCP components, such as sockets
     * @param buf Vector to collect data 
     */
    void connect(buffer_type& buf);
    
   /**
     * @brief recv fuction : discard
     */
    void recvData(sock_ptr sock);

    /**
     * @brief Handler of tcp connection
     */
    void connHandler(const error_code& ec, sock_ptr sock);

    /**
     * @brief Handler of tcp connection
     * @param buf Vector to collect data 
     * @param ec Default error code 
     * @param sock Socket pointer
     */
    void connHandler(buffer_type& buf, const error_code& ec, sock_ptr sock);
    
    /**
     * @brief Handler to read data in buffer
     * @param ec Error code of TCP socket
     */
    void readHandler(const error_code& ec);

    /**
     * @brief Invoked to print data in buffer
     */
    void testPrint(buffer_type& buf);

    /**
     * @brief Mavlink message unpacking
     */
    void testMavUnpack(buffer_type& buf);

private:
    boost::asio::io_service tcp_io_;
    buffer_type tcp_buf_; // data buffer
    endpoint_type tcp_ep_; // tcp socket endpoint
    socket_type *sock_t_;
    boost::shared_mutex listen_mutex_;
    boost::shared_mutex buf_mutex_;
    bool is_listen_; // is client should keep listen

    recv_proc_type recv_proc_; // data processor
};

#endif
