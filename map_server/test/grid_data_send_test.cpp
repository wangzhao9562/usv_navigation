/**
  ******************************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  grid_data_send_test.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2020/1/14
  * - Brief:     Node to test grid data sending
********************************************************************************************
**/

#include "ros/ros.h"
#include <SDL/SDL_image.h>

#include "map_server/tcp_server.h"
#include "map_server/grid_data_pack_protocol.h"

void testPrintArray(uint8_t* arr, size_t size){
	std::cout << "Print array: ";
	for(int ind = 0; ind < size; ++ind){
		std::cout << static_cast<int>(arr[ind]) << " ";
	}
	std::cout << std::endl;
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "grid_data_send_test");
	ros::NodeHandle nh;
	
	ROS_INFO("grid_data_send_test: prepare to create server");
	TCPServer tcp_s(16686);

	SDL_Surface* img;
	unsigned char* pixels;
	std::string file_name = "/home/wz9562/Documents/catkin_test/src/usv_navigation/map_server/test/testmap.png";	

	// Load image through SDL.
	if(!(img = IMG_Load(file_name.c_str()))){
		std::string err_msg = std::string("Failed to open image file: ") + std::string(file_name) + std::string(" ") + IMG_GetError();
		throw std::runtime_error(err_msg); 
	}

	int row_stride, n_channels, avg_channels;
	int color_sum;
	double color_avg;
	int alpha;

	// Get fundamental information of map
	size_t map_width = img->w;
	size_t map_height = img->h;

	// print information of loaded map
	ROS_INFO_STREAM("grid_data_send_test: width " << map_width <<
		" height " << map_height);
	
	row_stride = img->pitch;
	n_channels = img->format->BytesPerPixel;	
	avg_channels = n_channels;
	// avg_channels = n_channels - 1;

	pixels = (unsigned char*)(img->pixels);

	uint8_t grid_data[map_height * map_width];

	for(size_t h = 0; h < map_height; ++h){
		for(size_t w = 0; w < map_width; ++w){
			grid_data[h + w * map_height] = static_cast<uint8_t>(*(pixels + h * row_stride + w * n_channels));
		}
	}

	size_t map_size = map_height * map_width;
	
	size_t stack_size = map_size + 5;
	uint8_t grid_data_stack[stack_size];

	grid_data_stack[0] = GridDataPackProtocol::pack_head_;
	grid_data_stack[1] = GridDataPackProtocol::pack_sec_bit_;
	grid_data_stack[stack_size - 1] = GridDataPackProtocol::pack_tail_;
	std::cout << static_cast<int>(grid_data_stack[stack_size - 1]) << std::endl;
	grid_data_stack[2] = static_cast<uint8_t>(map_width);
	grid_data_stack[3] = static_cast<uint8_t>(map_height);

	uint8_t* copy_ptr = grid_data_stack;
	copy_ptr += 4;
	std::memcpy(copy_ptr, grid_data, map_size); // copy grid data into stack
	
	// print pixel array
	testPrintArray(grid_data_stack, stack_size);	

	try{
		ROS_INFO("grid_data_send_test: set buffer");
		tcp_s.setWriteBuffer(grid_data_stack, sizeof(grid_data_stack) / sizeof(uint8_t)); 
		ROS_INFO("grid_data_send_test: Finish buffer setting");
	}
	catch(std::exception& e){
		ROS_INFO("grid_data_send_test: Error to set write buffer!");
	}

	try{
		tcp_s.run();
	}
	catch(std::exception& e){
		ROS_ERROR("grid_data_send_test, Error to start tcp server");
	}

	return 0;
}
