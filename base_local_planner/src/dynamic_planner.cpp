/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  dynamic_planner.cpp
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/1/9
  * - Brief:     Clean local planner used TP method
  *****************************************************************************
*/

#include <base_local_planner/dynamic_planner.h>
#include <costmap_2d/footprint.h>
#include <string>
#include <sstream>
#include <math.h>
#include <angles/angles.h>
#include <boost/algorithm/string.hpp>

#include <ros/console.h>

//for computing path distance
#include <queue>
#include <climits>

using namespace std;
using namespace costmap_2d;

namespace base_local_planner{
	
	void DynamicPlanner::reconfigure(BaseLocalPlannerConfig& cfg){
		BaseLocalPlannerConfig config(cfg);
	
		boost::mutex::scoped_lock l(configuration_mutex_);
		
		kp_ = config.kp;
		ki_ = config.ki;
		kd_ = config.kd;

		obs_detect_dist_ = config.obs_detect_dist;
		ang_for_left_ = config.ang_for_left;
		ang_for_right_ = config.ang_for_right;

		follow_vel_ = config.follow_vel;

		vis_ang_for_left_ = config.vis_ang_for_left;
		vis_ang_for_right_ = config.vis_ang_for_right;
	}	

	DynamicPlanner::DynamicPlanner(WorldModel& world_model, 
		const Costmap2D& costmap,
		std::vector<geometry_msgs::Point> footprint_spec,
		double pdist_scale, double gdist_scale, double occdist_scale,
		double follow_vel, double ang_for_left, double ang_for_right,
		double obs_detect_dist, 
		double vis_ang_for_left, double vis_ang_for_right,
		double xy_goal_tolerance) : 
		path_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
		goal_map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()),
		costmap_(costmap), 
		world_model_(world_model), footprint_spec_(footprint_spec),
		pdist_scale_(pdist_scale), gdist_scale_(gdist_scale), occdist_scale_(occdist_scale),
		follow_vel_(follow_vel), 
		ang_for_left_(ang_for_left), ang_for_right_(ang_for_right),
		obs_detect_dist_(obs_detect_dist), vis_ang_for_left_(vis_ang_for_left), vis_ang_for_right_(vis_ang_for_right),
		follow_interrupt_dist_(xy_goal_tolerance)

		{
			// robot is not stuck to begin with
			avoid_count_ = 0;
			last_choice_ = AvoidOrientation::MID;
			
			costmap_2d::calculateMinAndMaxDistances(footprint_spec_, inscribed_radius_, circumscribed_radius_);   
		}

	//calculate the cost of a ray-traced line
	double DynamicPlanner::lineCost(int x0, int x1, int y0, int y1, bool mode){
		//Bresenham Ray-Tracing
		int deltax = abs(x1 - x0);        // The difference between the x's
		int deltay = abs(y1 - y0);        // The difference between the y's
		int x = x0;                       // Start x off at the first pixel
		int y = y0;                       // Start y off at the first pixel

		int xinc1, xinc2, yinc1, yinc2;
		int den, num, numadd, numpixels;

		double line_cost = 0.0;
		double point_cost = -1.0;

		if (x1 >= x0)                 // The x-values are increasing
		{
			xinc1 = 1;
			xinc2 = 1;
		}
		if (x1 >= x0)                 // The x-values are increasing
		{
			xinc1 = 1;
			xinc2 = 1;
		}
		else                          // The x-values are decreasing
		{
			xinc1 = -1;
			xinc2 = -1;
		}

		if (y1 >= y0)                 // The y-values are increasing
		{
			yinc1 = 1;
			yinc2 = 1;
		}
		else                          // The y-values are decreasing
		{
			yinc1 = -1;
			yinc2 = -1;
		}
		if (deltax >= deltay)         // There is at least one x-value for every y-value
		{
			xinc1 = 0;                  // Don't change the x when numerator >= denominator
			yinc2 = 0;                  // Don't change the y for every iteration
			den = deltax;
			num = deltax / 2;
			numadd = deltay;
			numpixels = deltax;         // There are more x-values than y-values
		} else {                      // There is at least one y-value for every x-value
			xinc2 = 0;                  // Don't change the x for every iteration
			yinc1 = 0;                  // Don't change the y when numerator >= denominator
			den = deltay;
			num = deltay / 2;
			numadd = deltax;
			numpixels = deltay;         // There are more y-values than x-values
		}

		// ROS_WARN("local_planner: start bresenham raytrace with %d points", numpixels);

		for (int curpixel = 0; curpixel <= numpixels; curpixel++) {
			point_cost = pointCost(x, y); //Score the current point

			if (point_cost < 0) {
				return -1;
			}

			if (line_cost < point_cost) {
				line_cost = point_cost;
			}

			num += numadd;              // Increase the numerator by the top of the fraction
			if (num >= den) {           // Check if numerator >= denominator
				num -= den;               // Calculate the new numerator value
				x += xinc1;               // Change the x as appropriate
				y += yinc1;               // Change the y as appropriate
			}
			x += xinc2;                 // Change the x as appropriate
			y += yinc2;                 // Change the y as appropriate
		}

		return line_cost;
	}

	double DynamicPlanner::pointCost(int x, int y){
		unsigned char cost = costmap_.getCost(x, y);
		//if the cell is in an obstacle the path is invalid
		if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION){
			return -1;
		}

		return cost;
	}

	void DynamicPlanner::updatePlan(const vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists){
		global_plan_.resize(new_plan.size());
		for(unsigned int i = 0; i < new_plan.size(); ++i){
			global_plan_[i] = new_plan[i];
		}

		if( global_plan_.size() > 0 ){
			geometry_msgs::PoseStamped& final_goal_pose = global_plan_[ global_plan_.size() - 1 ];
			final_goal_x_ = final_goal_pose.pose.position.x;
			final_goal_y_ = final_goal_pose.pose.position.y;
			final_goal_position_valid_ = true;
		} else {
			final_goal_position_valid_ = false;
		}

		if (compute_dists) {
			//reset the map for new operations
			path_map_.resetPathDist();
			goal_map_.resetPathDist();

			//make sure that we update our path based on the global plan and compute costs
			path_map_.setTargetCells(costmap_, global_plan_);
			goal_map_.setLocalGoal(costmap_, global_plan_);
			ROS_DEBUG("Path/Goal distance computed");
		}
	}

	double DynamicPlanner::footprintCost(double x_i, double y_i, double theta_i){
		//check if the footprint is legal
		return world_model_.footprintCost(x_i, y_i, theta_i, footprint_spec_, inscribed_radius_, circumscribed_radius_);
	}

	void DynamicPlanner::getLocalGoal(double& x, double& y){
		x = path_map_.goal_x_;
		y = path_map_.goal_y_;
	}



	bool DynamicPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
		MapCell cell = path_map_(cx, cy);
		MapCell goal_cell = goal_map_(cx, cy);

		if (cell.within_robot) {
			return false;
		}
		occ_cost = costmap_.getCost(cx, cy);

		if (cell.target_dist == path_map_.obstacleCosts() ||
			cell.target_dist == path_map_.unreachableCellCosts() ||
			occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
			return false;
		}
		path_cost = cell.target_dist;
		goal_cost = goal_cell.target_dist;
		total_cost = pdist_scale_ * path_cost + gdist_scale_ * goal_cost + occdist_scale_ * occ_cost;
		return true;
	}

	void DynamicPlanner::outputCostOfThreePath(double cur_pos_x, double cur_pos_y, double cur_pos_th, double& line_cost_of_mid, double& line_cost_of_left, double& line_cost_of_right){
		// ROS_INFO("local_planner: compute line cost of TP %f, %f, %f", cur_pos_x, cur_pos_y, cur_pos_th);

		unsigned int cur_pos_x_in_map, cur_pos_y_in_map;
		costmap_.worldToMap(cur_pos_x, cur_pos_y, cur_pos_x_in_map, cur_pos_y_in_map);

		double x_in_mid = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th);
		double y_in_mid = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th);
		unsigned int x_in_mid_in_map, y_in_mid_in_map;
		costmap_.worldToMap(x_in_mid, y_in_mid, x_in_mid_in_map, y_in_mid_in_map);
		line_cost_of_mid = lineCost(cur_pos_x_in_map, x_in_mid_in_map, cur_pos_y_in_map, y_in_mid_in_map, true);

		double ang_for_left_in_rad = ang_for_left_ * PI / 180;
		double x_in_left = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th + ang_for_left_in_rad) / std::cos(ang_for_left_in_rad);
		double y_in_left = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th + ang_for_left_in_rad) / std::cos(ang_for_left_in_rad);
		unsigned int x_in_left_in_map, y_in_left_in_map;
		costmap_.worldToMap(x_in_left, y_in_left, x_in_left_in_map, y_in_left_in_map);
		line_cost_of_left = lineCost(cur_pos_x_in_map, x_in_left_in_map, cur_pos_y_in_map, y_in_left_in_map, true);

		double ang_for_right_in_rad = ang_for_right_ * PI / 180;
		double x_in_right = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th - ang_for_right_in_rad) / std::cos(ang_for_right_in_rad);
		double y_in_right = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th - ang_for_right_in_rad) / std::cos(ang_for_right_in_rad);
		unsigned int x_in_right_in_map, y_in_right_in_map;
		costmap_.worldToMap(x_in_right, y_in_right, x_in_right_in_map, y_in_right_in_map);
		line_cost_of_right = lineCost(cur_pos_x_in_map, x_in_right_in_map, cur_pos_y_in_map, y_in_right_in_map, true);

		// ROS_INFO("local_planner: line cost of tp mid: %f, left: %f, right: %f", line_cost_of_mid, line_cost_of_left, line_cost_of_right);
	}

	bool DynamicPlanner::isFollowBlocked(double next_wp_x, double next_wp_y, double cur_pos_x, double cur_pos_y, double cur_pos_th ){
		// compute distance between currrent position of robot and next way point of planned path
		double delt_x = std::fabs(next_wp_x - cur_pos_x);
		double delt_y = std::fabs(next_wp_y - cur_pos_y);
		double dist_of_next_wp = std::sqrt(delt_x * delt_x + delt_y * delt_y);

		// if(dist_of_next_wp <= obs_detect_dist_){
		if(obs_detect_dist_ <= dist_of_next_wp){
			// ROS_INFO("local_planner: compute line cost of wp %f, %f current %f, %f, %f", next_wp_x, next_wp_y, cur_pos_x, cur_pos_y, cur_pos_th);

			// compute line cost, if cost of line is -1, the way is blocked by obs
			unsigned int cur_pos_x_in_map, cur_pos_y_in_map;
			unsigned int next_pos_x_in_map, next_pos_y_in_map;
			costmap_.worldToMap(cur_pos_x, cur_pos_y, cur_pos_x_in_map, cur_pos_y_in_map);
			costmap_.worldToMap(next_wp_x, next_wp_y, next_pos_x_in_map, next_pos_y_in_map);
			double cost_of_way = lineCost(cur_pos_x_in_map, next_pos_x_in_map, cur_pos_y_in_map, next_pos_y_in_map);

			return cost_of_way < 0;
		}
		else{
			double next_pos_x = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th);
			double next_pos_y = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th);
			// ROS_INFO("local_planner: compute line cost of mid way %f, %f current %f, %f, %f", next_pos_x, next_pos_y, cur_pos_x, cur_pos_y, cur_pos_th);
			unsigned int cur_pos_x_in_map, cur_pos_y_in_map;
			unsigned int next_pos_x_in_map, next_pos_y_in_map;
			costmap_.worldToMap(cur_pos_x, cur_pos_y, cur_pos_x_in_map, cur_pos_y_in_map);
			costmap_.worldToMap(next_pos_x, next_pos_y, next_pos_x_in_map, next_pos_y_in_map);
			double cost_of_way = lineCost(cur_pos_x_in_map, next_pos_x_in_map, cur_pos_y_in_map, next_pos_y_in_map);

			return cost_of_way < 0;
		}
	}

	bool DynamicPlanner::isFrontPathFree(double cur_pos_x, double cur_pos_y, double cur_pos_th){
		// compute line cost of mid way
		unsigned int cur_pos_x_in_map, cur_pos_y_in_map;
		costmap_.worldToMap(cur_pos_x, cur_pos_y, cur_pos_x_in_map, cur_pos_y_in_map);
		double x_in_mid = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th);
		double y_in_mid = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th);
		unsigned int x_in_mid_in_map, y_in_mid_in_map;
		costmap_.worldToMap(x_in_mid, y_in_mid, x_in_mid_in_map, y_in_mid_in_map);
		double line_cost_of_mid = lineCost(cur_pos_x_in_map, x_in_mid_in_map, cur_pos_y_in_map, y_in_mid_in_map, true);

		// compute line cost of left way of in visual scope
		double vis_ang_for_left_in_rad = vis_ang_for_left_ * PI / 180;
		double vis_x_in_left = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th + vis_ang_for_left_in_rad) / std::cos(vis_ang_for_left_in_rad);
		double vis_y_in_left = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th + vis_ang_for_left_in_rad) / std::cos(vis_ang_for_left_in_rad);
		unsigned int vis_x_in_left_in_map, vis_y_in_left_in_map;
		costmap_.worldToMap(vis_x_in_left, vis_y_in_left, vis_x_in_left_in_map, vis_y_in_left_in_map);
		double line_cost_of_vis_left = lineCost(cur_pos_x_in_map, vis_x_in_left_in_map, cur_pos_y_in_map, vis_y_in_left_in_map, true);

		// compute line cost of right way of in visual scope
		double vis_ang_for_right_in_rad = vis_ang_for_right_ * PI / 180;
		double vis_x_in_right = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th - vis_ang_for_right_in_rad) / std::cos(vis_ang_for_right_in_rad);
		double vis_y_in_right = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th - vis_ang_for_right_in_rad) / std::cos(vis_ang_for_right_in_rad);
		unsigned int vis_x_in_right_in_map, vis_y_in_right_in_map;
		costmap_.worldToMap(vis_x_in_right, vis_y_in_right, vis_x_in_right_in_map, vis_y_in_right_in_map);
		double line_cost_of_vis_right = lineCost(cur_pos_x_in_map, vis_x_in_right_in_map, cur_pos_y_in_map, vis_y_in_right_in_map, true);

		// ROS_INFO("local_planner: line cost of visual three path mid: %f, left: %f, right: %f", line_cost_of_mid, line_cost_of_vis_left, line_cost_of_vis_right);
		if(line_cost_of_mid >= 0){
			return true;
		}
		return false;
	}

	bool DynamicPlanner::pointFollowWithTPMethod(double cur_pos_x, double cur_pos_y, double cur_pos_th, double& vx, double& vy, double& vth, double& goal_x, double& goal_y){
		// comput cost of three path
		double line_cost_of_mid, line_cost_of_left, line_cost_of_right;
		outputCostOfThreePath(cur_pos_x, cur_pos_y, cur_pos_th, line_cost_of_mid, line_cost_of_left, line_cost_of_right);

		if(line_cost_of_mid == -1 && line_cost_of_left == -1 && line_cost_of_right == -1){
			return false;
		}

		// transform -1 to max value
		if(line_cost_of_mid < 0){
			line_cost_of_mid = static_cast<double>(INT_MAX);
		}
		if(line_cost_of_left < 0){
			line_cost_of_left = static_cast<double>(INT_MAX);
		}
		if(line_cost_of_right < 0){
			line_cost_of_right =  static_cast<double>(INT_MAX);
		}

		// get all possible terminal point
		double mid_x, mid_y, left_x, left_y, right_x, right_y;
		getObsDetectionArea(cur_pos_x, cur_pos_y, cur_pos_th, mid_x, mid_y, left_x, left_y, right_x, right_y);

		// choose strategy 
		if(line_cost_of_left < line_cost_of_right){
			if(line_cost_of_left < line_cost_of_mid){
				double th_of_left = cur_pos_th - ang_for_left_ * PI / 180;
				pointFollowLOS(cur_pos_th - th_of_left, vx, vy, vth);
				goal_x = left_x;
				goal_y = left_y;
				++avoid_count_;
				last_choice_ = AvoidOrientation::LEFT;
				return true;
			}
			else if (line_cost_of_left == line_cost_of_mid){
				if(avoid_count_){ 
					if(last_choice_ == AvoidOrientation::LEFT){
						double th_of_left = cur_pos_th - ang_for_left_ * PI / 180;
						pointFollowLOS(cur_pos_th - th_of_left, vx, vy, vth);
						goal_x = left_x;
						goal_y = left_y;
						++avoid_count_;
						last_choice_ = AvoidOrientation::LEFT;
							 return true;
					}
					else{
						vx = follow_vel_;
						vy = 0;  
						vth = 0; 
						goal_x = mid_x;
						goal_y = mid_y;
						++avoid_count_;
						last_choice_ = AvoidOrientation::MID;
						return true;
					}
				}
				else{
					vx = follow_vel_;
					vy = 0; 
					vth = 0; 
					goal_x = mid_x;
					goal_y = mid_y;
					++avoid_count_;
					last_choice_ = AvoidOrientation::MID;
					return true;
				}
			}
			else{
				vx = follow_vel_;
				vy = 0; 
				vth = 0; 
				goal_x = mid_x;
				goal_y = mid_y;
				++avoid_count_;
				last_choice_ = AvoidOrientation::MID;
				return true;
			}
		}
		else if(line_cost_of_left == line_cost_of_right){
			if(avoid_count_){
				if(last_choice_ == AvoidOrientation::RIGHT){
					double th_of_right = cur_pos_th + ang_for_right_ * PI / 180;
					pointFollowLOS(cur_pos_th - th_of_right, vx, vy, vth);
					goal_x = right_x;
					goal_y = right_y;
					++avoid_count_;
					last_choice_ = AvoidOrientation::RIGHT;
					return true;
				}
				else{
					double th_of_left = cur_pos_th - ang_for_left_ * PI / 180;
					pointFollowLOS(cur_pos_th - th_of_left, vx, vy, vth);
					goal_x = left_x;
					goal_y = left_y;
					++avoid_count_;
					last_choice_ = AvoidOrientation::LEFT;
					return true;
				}
			}
			else{
				double th_of_left = cur_pos_th - ang_for_left_ * PI / 180;
				pointFollowLOS(cur_pos_th - th_of_left, vx, vy, vth);
				goal_x = left_x;
				goal_y = left_y;
				++avoid_count_;
				last_choice_ = AvoidOrientation::LEFT;
				return true;
			}
		}
		else{
			if(line_cost_of_right < line_cost_of_mid){
				double th_of_right = cur_pos_th + ang_for_right_ * PI / 180;
				pointFollowLOS(cur_pos_th - th_of_right, vx, vy, vth);
				goal_x = right_x;
				goal_y = right_y;
				++avoid_count_;
				last_choice_ = AvoidOrientation::RIGHT;
				return true;
			}
			else if(line_cost_of_right == line_cost_of_mid){
				if(avoid_count_){
					if(last_choice_ == AvoidOrientation::RIGHT){
						double th_of_right = cur_pos_th + ang_for_right_ * PI / 180;
						pointFollowLOS(cur_pos_th - th_of_right, vx, vy, vth);
						goal_x = right_x;
						goal_y = right_y;
						++avoid_count_;
						last_choice_ = AvoidOrientation::RIGHT;
						return true;
					}
					else{
						vx = follow_vel_;
						vy = 0;
						vth = 0;
						goal_x = mid_x;
						goal_y = mid_y;
						++avoid_count_;
						last_choice_ = AvoidOrientation::MID;
						return true;
					}
				}
				else{
					vx = follow_vel_;
					vy = 0;
					vth = 0;
					goal_x = mid_x;
					goal_y = mid_y;
					++avoid_count_;
					last_choice_ = AvoidOrientation::MID;
					return true;
				}
			}
			else{
				vx = follow_vel_;
				vy = 0;
				vth = 0;
				goal_x = mid_x;
				goal_y = mid_y;
				++avoid_count_;
				last_choice_ = AvoidOrientation::MID;
				return true;
			}
		}
	}

	bool DynamicPlanner::pointFollow(double& goal_x, double& goal_y, double& goal_th,
			      double& x_i, double& y_i, double& th_i,
								  double& vx, double& vy, double& vth)
	{
		double distance = hypot(x_i - goal_x, y_i - goal_y);
		double mid_th = atan2(goal_y - y_i, goal_x - x_i);
		double delc_fin = mid_th - th_i;

		if(distance <= follow_interrupt_dist_){
			vx = 0;
			vy = 0;
			vth = 0;
			return true;
		}
		else{
			pointFollowLOS(delc_fin, vx, vy, vth);
			return false;
		}
	}
	
	void DynamicPlanner::pointFollowLOS(double theta_det, double& vx, double& vy, double& vth){
		theta_det_add_ += theta_det;

		if(fabs(theta_det) >= PI){
			if(theta_det > 0)
				theta_det -= 2.0 * PI;
			else
				theta_det += 2.0 * PI;
		}

		double R = kp_ * theta_det + ki_ * theta_det_add_ + kd_ * (theta_det_last_ - theta_det);
		theta_det_last_ = theta_det;

		if(R != 0){
			vx = 0.05;
			vy = 0;
			vth = R;
		}
		else{
			vx = 0.05;
			vy = 0;
			vth = 0;
		}
	}

	std::vector<geometry_msgs::PoseStamped> DynamicPlanner::getVisualArea(double cur_pos_x, double cur_pos_y, double cur_pos_th){
		double mid_x, mid_y, vis_left_x, vis_left_y, vis_right_x, vis_right_y;

		getVisualArea(cur_pos_x, cur_pos_y, cur_pos_th, mid_x, mid_y, vis_left_x, vis_left_y, vis_right_x, vis_right_y);

		// Create pose message, notice that the frame id of messages are not given
		geometry_msgs::PoseStamped mid, vis_left, vis_right;
		mid.pose.position.x = mid_x;
		mid.pose.position.y = mid_y;
		mid.pose.orientation.w = 1.0;
		vis_left.pose.position.x = vis_left_x;
		vis_left.pose.position.y = vis_left_y;
		vis_left.pose.orientation.w = 1.0;
		vis_right.pose.position.x = vis_right_x;
		vis_right.pose.position.y = vis_right_y;
		vis_right.pose.orientation.w = 1.0;

		std::vector<geometry_msgs::PoseStamped> vis_pose_vec;
		vis_pose_vec.push_back(mid);
		vis_pose_vec.push_back(vis_left);
		vis_pose_vec.push_back(vis_right);

		return vis_pose_vec;
	}

	std::vector<geometry_msgs::PoseStamped> DynamicPlanner::getObsDetectionArea(double cur_pos_x, double cur_pos_y, double cur_pos_th){
		double mid_x, mid_y, obs_detect_left_x, obs_detect_left_y, obs_detect_right_x, obs_detect_right_y;

		getObsDetectionArea(cur_pos_x, cur_pos_y, cur_pos_th, mid_x, mid_y, obs_detect_left_x, obs_detect_left_y, obs_detect_right_x, obs_detect_right_y);

		// Create pose message, notice that the frame id of messages are not given
		geometry_msgs::PoseStamped mid, obs_detect_left, obs_detect_right;
		mid.pose.position.x = mid_x;
		mid.pose.position.y = mid_y;
		mid.pose.orientation.w = 1.0;
		obs_detect_left.pose.position.x = obs_detect_left_x;
		obs_detect_left.pose.position.y = obs_detect_left_y;
		obs_detect_left.pose.orientation.w = 1.0;
		obs_detect_right.pose.position.x = obs_detect_right_x;
		obs_detect_right.pose.position.y = obs_detect_right_y;
		obs_detect_right.pose.orientation.w = 1.0;

		std::vector<geometry_msgs::PoseStamped> obs_detect_pose_vec;
		obs_detect_pose_vec.push_back(mid);
		obs_detect_pose_vec.push_back(obs_detect_left);
		obs_detect_pose_vec.push_back(obs_detect_right);

		return obs_detect_pose_vec;
	}

	void DynamicPlanner::getVisualArea(double cur_pos_x, double cur_pos_y, double cur_pos_th, double& mid_x, double& mid_y, double& vis_left_x, double& vis_left_y, double& vis_right_x, double& vis_right_y){
		mid_x = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th);
		mid_y = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th);

		double vis_ang_for_left_in_rad = vis_ang_for_left_ * PI / 180;
		vis_left_x = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th + vis_ang_for_left_in_rad) / std::cos(vis_ang_for_left_in_rad);
		vis_left_y = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th + vis_ang_for_left_in_rad) / std::cos(vis_ang_for_left_in_rad);

		double vis_ang_for_right_in_rad = vis_ang_for_right_ * PI / 180;
		vis_right_x = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th - vis_ang_for_right_in_rad) / std::cos(vis_ang_for_right_in_rad);
		vis_right_y = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th - vis_ang_for_right_in_rad) / std::cos(vis_ang_for_right_in_rad);

	}

	void DynamicPlanner::getObsDetectionArea(double cur_pos_x, double cur_pos_y, double cur_pos_th, double& mid_x, double& mid_y, double& left_x, double& left_y, double& right_x, double& right_y){
		mid_x = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th);
		mid_y = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th);

		double ang_for_left_in_rad = ang_for_left_ * PI / 180;
		left_x = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th + ang_for_left_in_rad) / std::cos(ang_for_left_in_rad);
		left_y = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th + ang_for_left_in_rad) / std::cos(ang_for_left_in_rad);

		double ang_for_right_in_rad = ang_for_right_ * PI / 180;
		right_x = cur_pos_x + obs_detect_dist_ * std::cos(cur_pos_th - ang_for_right_in_rad) / std::cos(ang_for_right_in_rad);
		right_y = cur_pos_y + obs_detect_dist_ * std::sin(cur_pos_th - ang_for_right_in_rad) / std::cos(ang_for_right_in_rad);
	}


}; // end of namespace
