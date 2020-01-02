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
        : map_name_(""), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free),
        tcp_c_(nullptr)
    {
        ros::NodeHandle private_nh("~");    
        
        std::string tcp_ip;
        int tcp_port;
        int buf_size;

        // Set parameters
        private_nh.param("tcp_ip", tcp_ip, std::string("127.0.0.1"));
        private_nh.param("tcp_port", tcp_port, 16384);
        private_nh.param("buffer_size", buf_size, 2073858);

        // Initialize tcp client
        try{
            tcp_c_ = new TCPClient(tcp_ip, tcp_port, static_cast<size_t>buf_size);
        }
        catch(std::exception& e){
            ROS_ERROR("map_server: cannot initialize tcp client!");
        }

        if(tcp_c_){
            try{
                tcp_c_->run();
            }
            catch(std::exception& e){
                ROS_ERROR("map_server: cannot run tcp client!");
            }
        }

        ROS_INFO("map_server: wait for map");      

        map_sub_ = private_nh.subscribe("map_info", 1, &MapLoader::mapCallback, this);
        map_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("map_info", 1, true);
        map_pub_ = private_nh.advertise<std_msgs::String>("map_path", 1, true);

	boost::thread map_recv(boost::bind(&MapLoader::mapTransform, this)); // create thread to listen tcp port
    };

    MapLoader::~MapLoader(){
        if(tcp_c_){
            tcp_c_->stopListen();
            delete tcp_c_;
        }
        tcp_c_ = nullptr;
    }

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
		if(map->data[i] == 0){
			fputc(254, out);
		}
		else if(map->data[i] == 255){
			fputc(000, out);
		}
		else{
			fputc(205, out);
		}
		/*
                if (map->data[i] >= 0 && map->data[i] <= threshold_free_) { //occ [0,0.1)
                    fputc(254, out);
                } else if (map->data[i] <= 100 && map->data[i] >= threshold_occupied_) { //occ (0.65,1]
                    fputc(000, out);
                } else { //occ [0.1,0.65]
                    fputc(205, out);
                }
		*/
            }
        }

        fclose(out);

        std::string mapmetadatafile = map_name_ + ".yaml";
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

        geometry_msgs::Quaternion orientation = map->info.origin.orientasync 指令tion;
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
	while(ros::ok()){
		/* Receive map info from workstation through boost TCP socket */
		std::vector<u_int8_t> map_info_buf;
		tcp_c_->getBuf(map_info_buf);
		/* Decode message through mavlink protocol */
		mavlink_message_t mav_msg; // mavlink message
		mavlink_status_t mav_status; // mavlink status
		// mavlink_map_info_t mav_map_info;

		std::string tp_map_name;
		u_int32_t map_width, map_height;
		std::vector<u_int16_t> map_grid_info;
		double map_resolution;
		std::pair<u_int32_t, u_int32_t> map_origin;
		std::pair<u_int32_t, u_int32_t> map_coord;

		// decode
		for(int ind = 0; ind < map_info_buf.size(); ++ind){
		    u_int8_t mav_c = map_info_buf[ind];
		    if(mavlink_parse_char(MAVLINK_COMM_0, mav_c, &mav_msg, &mav_status)){
			switch(mav_msg.msgid){
			    case MAP_INFO:
			    {
				// get map name 
				u_int8_t* tp_map_name;
				mavlink_msg_map_info_get_map_name(&mav_msg, tp_map_name);
				tp_map_name = std::string(tp_map_name);
				
				// get map grid information
				map_width = mavlink_msg_map_info_get_map_width(&mav_msg);
				map_height = mavlink_msg_map_info_get_map_height(&mav_msg);
				u_int32_t map_size = map_width * map_height;
				u_int_16_t tp_map_grid_info[map_size];
				mavlink_msg_map_info_get_occupancy_grid(&mav_msg, tp_map_grid_info);
				map_grid_info.insert(map_grid_info.end(), tp_map_grid_info, tp_map_grid_info + map_size);

				// get map resolution
				map_resolution = mavlink_msg_map_info_get_resolution(&map_msg);

				// get map origin point
				map_origin.first = mavlink_msg_map_info_get_origin_x(&mav_msg);
				map_origin.second = mavlink_msg_map_info_get_origin_y(&mav_msg);

				// get map coordinate in last map
				map_coord.first = mavlink_msg_map_info_get_x_in_last_map(&mav_msg);
				map_coord.second = mavlink_msg_map_info_get_y_in_last_map(&mav_msg);
				break; 
			    }
			    default:
				break;
			}
		    }
		}
		/* Transform map info into formation of nav_msgs::OccupancyGrid and publish it on innter topic*/
		if(map_name != tp_map_name){
			map_name_ = tp_map_name;

			nav_msgs::OccupancyGrid map_info_msg;
			map_info_msg.header.frame_id = "map";
			map_info_msg.header.stamp = ros::Time::now();

			map_info_msg.info.resolution = map_resolution;
			map_info_msg.info.width = map_width;
			map_info_msg.info.height = map_height;
			map_info_msg.info.origin.position.x = map_origin.first;
			map_info_msg.info.origin.position.y = map_origin.second;
			map_info_msg.info.origin.orientation.w = 1.0;	

			map_info_msg.data = map_info_buf; // not sure

			map_info_pub_.publish(map_info_buf); // publish
		}
		boost::this_thread.sleep(boost::posix_time::second(1));
	}
    };
}; // end of namespace
