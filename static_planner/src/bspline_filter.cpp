/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  bspline_filter.cpp
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/9
  * - Brief:     Bspline interpolation filter for global planned path
  ******************************************************************************
  * History:
  * 2019/12/9 
  * Complete implementation of BsplineFilter, complie successfully
  ******************************************************************************
*/

#include "static_planner/bspline_filter.h"
#include <fstream>

namespace static_planner{

void BsplineFilter::processPath(const geometry_msgs::PoseStamped& start, std::vector<geometry_msgs::PoseStamped>& path){
  recordPath(path, false); // test

  if(path.size() > 0){
    std::vector<geometry_msgs::PoseStamped> whole_interpolated_path;
 
    // Create whole complete path
    std::vector<geometry_msgs::PoseStamped> whole_path;
    whole_path.push_back(start);
    whole_path.insert(whole_path.end(), path.begin(), path.end());

    // Remove point in line
    removeInLinePoint(whole_path);

    // Interpolation
    for(int index = 0; index < whole_path.size() - 2; ++index){
      std::vector<geometry_msgs::PoseStamped> result = quadricBsplineInterpolation(whole_path[index], whole_path[index + 1], whole_path[index + 2], dt_);
      whole_interpolated_path.insert(whole_interpolated_path.end(), result.begin(), result.end());
      if(index == whole_path.size() - 3){
        whole_interpolated_path.push_back(path[path.size() - 1]);
      } 
    }
    
    path = whole_interpolated_path;
    recordPath(path, true); // test
  }
}
 

void BsplineFilter::removeInLinePoint(std::vector<geometry_msgs::PoseStamped>& path){
  if(path.size() >= 3){
    std::vector<int> del_list;

    // Find points has same orientation
    for(int index = 0; index < path.size() - 1; ++index){
      double yaw = tf::getYaw(path[index].pose.orientation);
      int pos = index;
      for(int inner_ind = index + 1; inner_ind < path.size(); ++inner_ind){
        if(std::fabs(tf::getYaw(path[inner_ind].pose.orientation) - yaw) < 0.0001){
          pos = inner_ind;
        }
        else{
          break;
        }
      }
      if(pos - index > 1){
        for(int del_ind = index + 1; del_ind < pos; ++del_ind){
          del_list.push_back(del_ind);
        }
      }
    }

   
    // Remove  
    std::vector<geometry_msgs::PoseStamped> filtered_path;
    for(int index = 0; index < path.size(); ++index){
      if(std::find(del_list.begin(), del_list.end(), index) == del_list.end()){
        filtered_path.push_back(path[index]);
      }
    }

    path = filtered_path;
  }  
}

std::vector<geometry_msgs::PoseStamped> BsplineFilter::quadricBsplineInterpolation(geometry_msgs::PoseStamped point1, geometry_msgs::PoseStamped point2, geometry_msgs::PoseStamped point3, double dt){
  std::vector<geometry_msgs::PoseStamped> bspline_path;
  std::vector<std::pair<double, double>> coordinate;
  std::vector<double> orientation;

  // Get coordinate and orientation of points
  double x1 = point1.pose.position.x;
  double y1 = point1.pose.position.y;
  double x2 = point2.pose.position.x;
  double y2 = point2.pose.position.y;
  double x3 = point3.pose.position.x;
  double y3 = point3.pose.position.y;

  double h1 = tf::getYaw(point1.pose.orientation);
  double h2 = tf::getYaw(point2.pose.orientation);
  double h3 = tf::getYaw(point3.pose.orientation);

  // Get coordinate and orientation of interpolated points
  quadricBsplineInterpolation(x1, y1, h1, x2, y2, h2, x3, y3, h3, dt, coordinate, orientation);

  // Pack points into geometry_msgs
  for(int index = 0; index < coordinate.size(); ++index){
    geometry_msgs::PoseStamped interpolated_point;
    interpolated_point.pose.position.x = coordinate[index].first;
    interpolated_point.pose.position.y = coordinate[index].second;
    interpolated_point.pose.orientation = tf::createQuaternionMsgFromYaw(orientation[index]);
    bspline_path.push_back(interpolated_point);
  }

  return bspline_path;
}

void BsplineFilter::quadricBsplineInterpolation(double x1, double y1, double h1, double x2, double y2, double h2, double x3, double y3, double h3, double dt, std::vector<std::pair<double, double>>& coordinate, std::vector<double>& orientation){ 
  double g1, g2, g3;
  double p_x, p_y, p_h;

  // coordinate.push_back(std::make_pair(x1, y1));
  // orientation.push_back(h1);

  double previous_x = x1;
  double previous_y = y1;
  double previous_h = h1;

  double t = 0;

  while(t <= 1){
    // Calculate fundamental function
    g1 = 0.5 * (t - 1) * (t - 1);
    g2 = 0.5 * (-2 * t * t + 2 * t + 1);
    g3 = 0.5 * t * t;

    // Calculate interpolated point
    p_x = x1 * g1 + x2 * g2 + x3 * g3;
    p_y = y1 * g1 + y2 * g2 + y3 * g3;
    // p_h = std::atan2(p_y - coordinate[coordinate.size() - 1].second, p_x - coordinate[coordinate.size() - 1].first);
    p_h = std::atan2(p_y - previous_y, p_y - previous_x); 

    // if(p_x != coordinate[coordinate.size() - 1].first || p_y != coordinate[coordinate.size() - 1].second){
    //   coordinate.push_back(std::make_pair(p_x, p_y));
    //   orientation.push_back(p_h);
    // }
  
    coordinate.push_back(std::make_pair(p_x, p_y));
    orientation.push_back(p_h);

    previous_x = p_x; 
    previous_y = p_y;
    previous_h = p_h;

    t += dt; 
  }
}

void BsplineFilter::recordPath(std::vector<geometry_msgs::PoseStamped> path, bool is_filtered = false){
  std::string filename;
  if(is_filtered){
    filename = "/home/wz9562/Documents/my_files/filtered_path.txt";
  }
  else{
    filename = "/home/wz9562/Documents/my_files/origin_path.txt";
  }
  std::fstream f;
  f.open(filename);
  if(f.is_open()){
    std::for_each(path.begin(), path.end(), [&f](geometry_msgs::PoseStamped point){
      f << point.pose.position.x << "," << point.pose.position.y << " ";
    });
  }
  f.close();
}

}; // end namespace static_planner
