/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  bspline_filter.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/9
  * - Brief:     Bspline interpolation filter for global planned path
  ******************************************************************************
  * History:
  * 2019/12/9 
  * Complete definition of BsplineFilter, complie successfully
  ******************************************************************************
*/

#ifndef BSPLINE_FILTER_H_
#define BSPLINE_FILTER_H_

#include "static_planner/orientation_filter.h"
#include <cmath>
#include <float.h>
#include <algorithm>
#include <utility>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

#include <tf/transform_datatypes.h>

namespace static_planner {

class BsplineFilter : public OrientationFilter{
public:
  /**
   * @brief Default constructor
   */
  BsplineFilter() : OrientationFilter(){ dt_ = 0.2; };

  /**
   * @biref Constructor
   * @param dt Interval of interpolation
   */
  BsplineFilter(double dt) : OrientationFilter(), dt_(dt){}

  /**
   * @brief Deconstructor
   */
  ~BsplineFilter(){};
 
  /**
   * @brief Process of planned global path
   * @param start Start point
   * @param path Planned global path
   */
  void processPath(const geometry_msgs::PoseStamped& start,
                   std::vector<geometry_msgs::PoseStamped>& path)override;

private:
  /**
   * @brief Remove point in line of planned path
   * @param path Planned global path
   */
  void removeInLinePoint(std::vector<geometry_msgs::PoseStamped>& path);

  /**
   * @brief Interpolate through bspline algorithm
   * @param point1 First interpolated point
   * @param point2 Second interpolated point
   * @param point3 Third interpolated point
   * @param dt Interpolation interval
   * @return Interpolation result
   */
  std::vector<geometry_msgs::PoseStamped> quadricBsplineInterpolation(geometry_msgs::PoseStamped point1, 
                            geometry_msgs::PoseStamped point2, 
                            geometry_msgs::PoseStamped point3, double dt);

  /**
   * @brief Interpolate through bspline algorithm
   * @param x1 x coordinate of first interpolation point 
   * @param y1 y coordinate of first interpolation point 
   * @param h1 Orientation of first interpolation point
   * @param x2 x coordinate of second interpolation point 
   * @param y2 y coordinate of second interpolation point 
   * @param h2 Orientation of second interpolation point
   * @param x3 x coordinate of third interpolation point 
   * @param y3 y coordinate of third interpolation point
   * @param h3 Orientation of third interpolation point
   * @param coordinate Coordination of interpolated points
   * @param orientation Orientation of each interpolated points
   */
  void quadricBsplineInterpolation(double x1, double y1, double h1, 
                                   double x2, double y2, double h2,
                                   double x3, double y3, double h3, 
                                   double dt, std::vector<std::pair<double, double>>& coordinate, std::vector<double>& orientation);

  /**
   * @brief Record original path or filtered path 
   * @param path Plan to record
   * @param is_filtered Is input plan filtered
   */
  void recordPath(std::vector<geometry_msgs::PoseStamped> path, bool is_filtered);  

private:
   double dt_; // Interpolation interval
};

} // end namespace static_planner
#endif
