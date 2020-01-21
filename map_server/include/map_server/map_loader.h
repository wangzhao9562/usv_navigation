/**
  *******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  map_loader.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/12/30
  * - Brief:     Definition of MapLoader class to load map information from control station
  *******************************************************************************
 **/

#ifndef MAP_LOADER_H_
#define MAP_LOADER_H_

#include <cstdio>

#include "map_server/mavlink/v2.0/arms_usv_nav/mavlink.h"
#include "map_server/tcp_client.h"
#include "map_server/MapServerInfo.h"

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "nav_msgs/GetMap.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

namespace map_server{
    /**
     * @brief Component to load map information from work station and transfer it into map 
     * file in form of .pgm and .yaml 
     */
    class MapLoader{
    public:
        /**
         * @brief Constructor of MapLoader
         * @param save_map if save map 
         * @param map_path path to store map
         * @param threshold_occupied Threshold value for occupied grid
         * @param threshold_free Threshold value for free grid
         */
        MapLoader(std::string map_path, bool save_map, int threshold_occupied, int threshold_free);

        /**
         * @brief Deconstructor of MapLoader
         */
        ~MapLoader();

    private:
        /**
         * @brief Callback function to receive grid information of map
         * @param map Grid information of map in form of nav_msgs::OccupancyGrid
         */
        void mapCallback(const nav_msgs::OccupancyGridConstPtr& map);

        /**
         * @brief Receive grid information of map from work station like QGC, then transform the info
         *  into form of nav_msgs::OccupancyGrid and publish.
         */
        void mapTransform();

    private:
        std::string map_path_;
        std::string map_name_; 
        
	int threshold_occupied_; 
        int threshold_free_;
	
        ros::Subscriber map_sub_; // ros subscriber which receive map information from ros topic
        ros::Publisher map_info_pub_; // inner ros publisher  

	ros::Publisher map_s_info_pub_; 

        TCPClient* map_info_tcp_c_; // tcp client for map information
	TCPClient* grid_data_tcp_c_; // tcp client for map grid data
    };
} // end of namespace


#endif
