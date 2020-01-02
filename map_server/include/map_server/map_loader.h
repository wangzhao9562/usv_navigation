/**
  ******************************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  map_loader.h
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2019/12/30
  * - Brief:     Definition of MapLoader class to load map information from control station
********************************************************************************************
**/

#ifndef MAP_LOADER_H_
#define MAP_LOADER_H_

#include <cstdio>

#include "map_server/mavlink/v2.0/ARMsUsvNav/mavlink.h"
#include "map_server/tcp_client.h"

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
         * @param threshold_occupied Threshold value for occupied grid
         * @param threshold_free Threshold value for free grid
         */
        MapLoader(bool save_map, int threshold_occupied = 100, int threshold_free = 0);

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
        std::string map_name_; 
        int threshold_occupied_; 
        int threshold_free_;

        ros::Subscriber map_sub_; // ros subscriber which receive map information from ros topic
        ros::Publisher map_info_pub_; // inner ros publisher  

        TCPClient* tcp_c_;
    };
} // end of namespace


#endif