/*
 * @Author: your name
 * @Date: 2020-01-02 09:40:01
 * @LastEditTime : 2020-01-02 09:57:42
 * @LastEditors  : Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /usv_navigation/map_server/src/map_loader.cpp
 */
/**
  ******************************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  map_loader.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2020/1/2
  * - Brief:     Implementation of MapLoader class to load map information from control station
********************************************************************************************
**/

#include "map_server/map_loader.h"

namespace map_server{
    MapLoader::MapLoader(bool save_map, int threshold_occupied = 100, int threshold_free = 0) 
        : map_name_(""), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free)
    {
        ros::NodeHandle private_nh("~");    

        ROS_INFO("map_server: wait for map");      

        map_sub_ = private_nh.subscribe("map_info", 1, &MapLoader::mapCallback, this);
        map_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("map_info", 1, true);
        map_pub_ = private_nh.advertise<std_msgs::String>("map_path", 1, true);
    };

    void MapLoader::mapCallback(const nav_msgs::OccupancyGridConstPtr& map){
        ROS_INFO("map_server: Received a %d X %d map @ %.3f m/pix",
               map->info.width,
               map->info.height,
               map->info.resolution);


        std::string mapdatafile = map_name_ + ".pgm";
        ROS_INFO("map_server: Writing map occupancy data to %s", mapdatafile.c_str());
        FILE* out = fopen(mapdatafile.c_str(), "w");
        if (!out)
        {
            ROS_ERROR("map_server: Couldn't save map file to %s", mapdatafile.c_str());
            return;
        }

        fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
              map->info.resolution, map->info.width, map->info.height);
        for(unsigned int y = 0; y < map->info.height; y++) {
            for(unsigned int x = 0; x < map->info.width; x++) {
                unsigned int i = x + (map->info.height - y - 1) * map->info.width;
                if (map->data[i] >= 0 && map->data[i] <= threshold_free_) { //occ [0,0.1)
                    fputc(254, out);
                } else if (map->data[i] <= 100 && map->data[i] >= threshold_occupied_) { //occ (0.65,1]
                    fputc(000, out);
                } else { //occ [0.1,0.65]
                    fputc(205, out);
                }
            }
        }

        fclose(out);

        std::string mapmetadatafile = mapname_ + ".yaml";
        ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
        FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

        /*
        resolution: 0.100000
        origin: [0.000000, 0.000000, 0.000000]
        #
        negate: 0
        occupied_thresh: 0.65
        free_thresh: 0.196
        */

        geometry_msgs::Quaternion orientation = map->info.origin.orientation;
        tf2::Matrix3x3 mat(tf2::Quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ));
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);

        fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
              mapdatafile.c_str(), map->info.resolution, map->info.origin.position.x, map->info.origin.position.y, yaw);

        fclose(yaml);
        ROS_INFO("map_server: Done\n");
    };

    void MapLoader::mapTransform(){

    };
}; // end of namespace