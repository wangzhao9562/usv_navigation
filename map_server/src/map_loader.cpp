/**
  *******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  map_loader.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2020/1/2
  * - Brief:     Implementation of MapLoader class to load map information from control station
  *******************************************************************************
**/

#include "map_server/map_loader.h"
#include "map_server/grid_data_pack_protocol.h"

namespace map_server{
    MapLoader::MapLoader(bool save_map, int threshold_occupied = 100, int threshold_free = 0) 
        : map_name_(""), threshold_occupied_(threshold_occupied), threshold_free_(threshold_free),
        map_info_tcp_c_(NULL)
    {
	ros::NodeHandle nh;
        ros::NodeHandle private_nh("~/map_loader");    
        
        std::string map_info_tcp_ip, grid_data_tcp_ip;
        int map_info_tcp_port, grid_data_tcp_port;

        // Set parameters
        private_nh.param("map_info_tcp_ip", map_info_tcp_ip, std::string("127.0.0.1"));
        private_nh.param("map_info_tcp_port", map_info_tcp_port, 6688);

	private_nh.param("grid_data_tcp_ip", grid_data_tcp_ip, std::string("127.0.0.1"));
	private_nh.param("grid_data_tcp_ip", grid_data_tcp_port, 6689);

        // Initialize tcp client for map information
        try{
            map_info_tcp_c_ = new TCPClient(map_info_tcp_ip, map_info_tcp_port);
        }
        catch(std::exception& e){
            ROS_ERROR("map_server: initialize map info tcp client failed!");
        }

	// Initialize tcp client for map grid data
	try{
	    grid_data_tcp_c_ = new TCPClient(grid_data_tcp_ip, grid_data_tcp_port);
	}
	catch(std::exception& e){
	    ROS_ERROR("map_server: initialize grid data tcp client failed!");
	}

	// Run map info tcp client
        if(map_info_tcp_c_){
            try{
                map_info_tcp_c_->run();
            }
            catch(std::exception& e){
                ROS_ERROR("map_server: run map info tcp client failed!");
            }
        }

	// Run grid data tcp client
	if(grid_data_tcp_c_){
	    try{
	        grid_data_tcp_c_->run();
	    }
	    catch(std::exception& e){
	        ROS_ERROR("map_server: run map grid data tcp client failed!");
	    }
	}

        ROS_INFO("map_server: wait for map");      

        map_sub_ = private_nh.subscribe("map_info", 1, &MapLoader::mapCallback, this);
        map_info_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("map_info", 1, true);
        map_path_pub_ = nh.advertise<std_msgs::String>("map_path", 1, true);

	boost::thread map_recv(boost::bind(&MapLoader::mapTransform, this)); // create thread to listen tcp port
    };

    MapLoader::~MapLoader(){
        if(map_info_tcp_c_){
            map_info_tcp_c_->stopListen();
            delete map_info_tcp_c_;
        }
        map_info_tcp_c_ = NULL;
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
	while(ros::ok()){
		/* Receive map info from workstation through boost TCP socket */
		std::vector<uint8_t> map_info_buf;
		map_info_tcp_c_->getBuf(map_info_buf);

		std::vector<uint8_t> grid_data_buf;
		grid_data_tcp_c_->getBuf(grid_data_buf);

		/* Decode message through mavlink protocol */
		mavlink_message_t mav_msg; // mavlink message
		mavlink_status_t mav_status; // mavlink status

		std::string tp_map_name;
		uint32_t map_width{0}, map_height{0};
		std::vector<uint32_t> map_grid_info;
		float map_resolution{0};
		std::pair<uint32_t, uint32_t> map_origin;
		std::pair<uint32_t, uint32_t> map_coord;

		// decode
		for(int ind = 0; ind < map_info_buf.size(); ++ind){
		    u_int8_t mav_c = map_info_buf[ind];
		    if(mavlink_parse_char(MAVLINK_COMM_0, mav_c, &mav_msg, &mav_status)){
			switch(mav_msg.msgid){
			    case MAVLINK_MSG_ID_MAP_INFO:
			    {
				// get map name 
				uint8_t* tp_map_name_in_char;
				mavlink_msg_map_info_get_map_name(&mav_msg, tp_map_name_in_char);
				tp_map_name = reinterpret_cast<char*>(const_cast<uint8_t*>(tp_map_name_in_char));

				// get map grid information
				map_width = mavlink_msg_map_info_get_map_width(&mav_msg);
				map_height = mavlink_msg_map_info_get_map_height(&mav_msg);
				uint32_t map_size = map_width * map_height;
				
				/*
				uint32_t tp_map_grid_info[map_size];
				mavlink_msg_map_info_get_occupancy_grid(&mav_msg, tp_map_grid_info);
				map_grid_info.insert(map_grid_info.end(), tp_map_grid_info, tp_map_grid_info + map_size);
				*/

				// get map resolution
				map_resolution = mavlink_msg_map_info_get_resolution(&mav_msg);

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
	
		/* Decode map grid data */
		std::vector<int> grid_data; // vector only contains map grid data

		for(int ind = 0; ind < grid_data_buf.size(); ++ind){
		    if(static_cast<char>(grid_data_buf[ind]) == GridDataPackProtocol::pack_head_){
		        for(int b_ind = ind + 1; b_ind < grid_data_buf.size(); ++b_ind){
			    if(static_cast<char>(grid_data_buf[b_ind]) == GridDataPackProtocol::pack_tail_){
			        int sec_bit = ind + 1;
			        int w_bit = ind + 2;
			        int h_bit = ind + 3;
			        
			        if(h_bit < grid_data_buf.size()){
				    // check if map size in stack is same as map info
				    if(static_cast<int>(grid_data_buf[w_bit] == map_height)
				&& static_cast<int>(grid_data_buf[h_bit] == map_width)
				&& (b_ind - ind == map_height * map_width + 4)){
				        for(int c_ind = h_bit + 1; c_ind < b_ind; ++c_ind){
					    grid_data.push_back(grid_data_buf[c_ind]);
					}
				    }
				}

			        break;
			    }
			}
		    }
		}

		/* Transform map info into formation of nav_msgs::OccupancyGrid and publish it on innter topic*/
		
		if(map_name_ != tp_map_name){
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

			// map_info_msg.data = map_info_buf; // not sure

			for(int ind = 0; ind < grid_data.size(); ++ind){
				map_info_msg.data[ind] = grid_data[ind];
			} // not sure

			map_info_pub_.publish(map_info_msg); // publish
			// map_path_pub_.publish(map);
		}
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
    };
}; // end of namespace
