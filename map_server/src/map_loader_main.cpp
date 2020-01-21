/**
  *******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  map_loader_main.cpp
  * - Author:    Zhao Wang
  * - Version:   1.0.0
  * - Date:      2020/1/17
  * - Brief:     Node to test map loader
  *******************************************************************************
**/

#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#include "map_server/map_loader.h"
#include "map_server/map_server.hpp"
#include "map_server/MapServerInfo.h"

void mapPathCallback(map_server::MapServerInfo::ConstPtr& msg){
     /* Invoke map server to load map */
}

int main(int argc, char* argv[]){
    ros::init(argc, argv, "map_server");
    ros::NodeHandle nh;    

    ros::Subscriber map_path_sub = nh.subscribe("map_path", 1,
		boost::bind(&mapPathCallback, _1));

    MapLoader ml(true);

    ros::spin();

    return 0;
}

