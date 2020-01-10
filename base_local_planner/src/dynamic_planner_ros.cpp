/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  dynamic_planner_ros.cpp
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/1/9
  * - Brief:     ROS wrapper class of clean local planner used TP method
  *****************************************************************************
*/

#include <base_local_planner/dynamic_planner_ros.h>

#include <sys/time.h>

#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>

// test
#include <sstream>
#include <std_msgs/String.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(base_local_planner::DynamicPlannerROS, nav_core::BaseLocalPlanner)

namespace base_local_planner{
	void DynamicPlannerROS::reconfigureCB(BaseLocalPlannerConfig &config, uint32_t level) {
		if (setup_ && config.restore_defaults) {
			config = default_config_;
			//Avoid looping
			config.restore_defaults = false;
		}
		if ( ! setup_) {
			default_config_ = config;
			setup_ = true;
		}
		tc_->reconfigure(config);
		reached_goal_ = false;
	}

	DynamicPlannerROS::DynamicPlannerROS() :
		world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom") {}

	DynamicPlannerROS::DynamicPlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros) :
		world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom") {

		//initialize the planner
		initialize(name, tf, costmap_ros);
	}

	void DynamicPlannerROS::initialize(std::string name,
		tf::TransformListener* tf, 
		costmap_2d::Costmap2DROS* costmap_ros)
	{
		if(!isInitialized()){
			ros::NodeHandle private_nh("~/" + name);
			g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
			l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
			// rviz
			test_mid_pub_ = private_nh.advertise<visualization_msgs::Marker>("mid_way", 1);
			test_vis_left_pub_ = private_nh.advertise<visualization_msgs::Marker>("vis_left_way", 1);
			test_vis_right_pub_ = private_nh.advertise<visualization_msgs::Marker>("vis_right_way", 1);
			test_obs_detect_left_pub_ = private_nh.advertise<visualization_msgs::Marker>("obs_detect_left_way", 1);
			test_obs_detect_right_pub_ = private_nh.advertise<visualization_msgs::Marker>("obs_detect_right_way", 1);
		
			tf_ = tf;
			costmap_ros_ = costmap_ros;
			std::string world_model_type;
			
			costmap_ = costmap_ros_->getCostmap();

			global_frame_ = costmap_ros_->getGlobalFrameID();
			robot_base_frame_ = costmap_ros_->getBaseFrameID();
			
			reached_goal_ = false;

			//declare
			private_nh.param("world_model", world_model_type, std::string("costmap"));

		        //parameters for using the freespace controller
		        double min_pt_separation, max_obstacle_height, grid_resolution;
		        private_nh.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
		        private_nh.param("point_grid/min_pt_separation", min_pt_separation, 0.01);
		        private_nh.param("point_grid/max_obstacle_height", max_obstacle_height, 2.0);
		        private_nh.param("point_grid/grid_resolution", grid_resolution, 0.2);

			// parameters for tp method
			double follow_vel, obs_detect_dist;
			double ang_for_left, ang_for_right;
			double vis_ang_for_left, vis_ang_for_right;

			// TP method parameters initialize
			private_nh.param("follow_vel", follow_vel, 0.5);
			private_nh.param("obs_detect_dist", obs_detect_dist, 0.5);
			private_nh.param("ang_for_left", ang_for_left, 45.0);
			private_nh.param("ang_for_right", ang_for_right, 45.0);
			private_nh.param("vis_ang_for_left", vis_ang_for_left, 5.0);
			private_nh.param("vis_ang_for_right", vis_ang_for_right, 5.0);
			private_nh.param("xy_goal_tolerance", xy_goal_tolerance_,0.5);
			private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_,0.087);
			private_nh.param("usv_mode", is_usv_mode_, false);

			// world model initialize
			ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");
			world_model_ = new CostmapModel(*costmap_);
	
			footprint_spec_ = costmap_ros_->getRobotFootprint();

			// local planner initialize
			tc_ = new DynamicPlanner(*world_model_, 
				*costmap_, footprint_spec_, follow_vel, 
				ang_for_left, ang_for_right, obs_detect_dist,
				vis_ang_for_left, vis_ang_for_right,
				xy_goal_tolerance_);

			map_viz_.initialize(name, global_frame_, 
				boost::bind(&DynamicPlanner::getCellCosts, tc_, _1, _2, _3, _4, _5, _6));
			initialized_ = true;
		
 			// set dynamic configure	
			dsrv_ = new dynamic_reconfigure::Server<BaseLocalPlannerConfig>(private_nh);
			dynamic_reconfigure::Server<BaseLocalPlannerConfig>::CallbackType cb = boost::bind(&DynamicPlannerROS::reconfigureCB, this, _1, _2);
			dsrv_->setCallback(cb);

		}
		else{
			ROS_WARN("base_local_planner", "This planner has already been initialized, doing nothing");
		}
	}

	DynamicPlannerROS::~DynamicPlannerROS() {
		//make sure to clean things up
		delete dsrv_;

		if(tc_ != NULL)
			delete tc_;

		if(world_model_ != NULL)
			delete world_model_;
	}

	bool DynamicPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
		if (! isInitialized()) {
			ROS_ERROR("base_local_planner", "This planner has not been initialized");
			return false;
		}

		//reset the global plan
		global_plan_.clear();
		global_plan_ = orig_global_plan;

		//reset the at goal flag
		reached_goal_ = false;
		return true;
	}

	bool DynamicPlannerROS::isFollowBlocked(unsigned int following_wp_num){
		if (! isInitialized()){
			ROS_ERROR("base_local_planner", "This planner has not been initialized");
			return false;
		}

		// get pose in global frame
		tf::Stamped<tf::Pose> global_pose;
		if(!costmap_ros_->getRobotPose(global_pose)){
			return false;
		}
		
		// get global plan in global frame
		std::vector<geometry_msgs::PoseStamped> transformed_plan;
		if(!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)){
			ROS_WARN("base_local_planner", "Could not transform the global plan to the frame of the controller");
			return false;
		}
		
		//if the global plan passed in is empty... we won't do anything
		if(transformed_plan.empty()){
			return false;
		}

		if(following_wp_num < transformed_plan.size()){
			tf::Stamped<tf::Pose> goal_point;

			try{
				tf::poseStampedMsgToTF(transformed_plan[following_wp_num], goal_point);
			}
			catch(std::exception& e){
				ROS_ERROR("base_local_planner", "transfer pose to tf msg from pose msg failed");
			}
			//we assume the global goal is the last point in the global plan
			double goal_x = goal_point.getOrigin().getX();
			double goal_y = goal_point.getOrigin().getY();

			// get current position of robot
			double pos_x = global_pose.getOrigin().getX();
			double pos_y = global_pose.getOrigin().getY();
			double pos_th = tf::getYaw(global_pose.getRotation());

			// rviz interface
			geometry_msgs::PoseStamped mid, vis_left, vis_right;
			std::vector<geometry_msgs::PoseStamped> vis_vec = tc_->getVisualArea(pos_x, pos_y, pos_th);
			vis_vec[0].header.frame_id = global_frame_;
			vis_vec[1].header.frame_id = global_frame_;
			vis_vec[2].header.frame_id = global_frame_;
			tf_->transformPose("odom", vis_vec[0], mid); // world frame to base of robot
			tf_->transformPose("odom", vis_vec[1], vis_left);
			tf_->transformPose("odom", vis_vec[2], vis_right);
			visualization_msgs::Marker mid_line, vis_left_line, vis_right_line;
			getLineVisMsgs(pos_x, pos_y, mid, mid_line, VisAreaType::VISUALAREA);
			getLineVisMsgs(pos_x, pos_y, vis_left, vis_left_line, VisAreaType::VISUALAREA);
			getLineVisMsgs(pos_x, pos_y, vis_right, vis_right_line, VisAreaType::VISUALAREA);
			
			test_mid_pub_.publish(mid_line);
			test_vis_left_pub_.publish(vis_left_line);
			test_vis_right_pub_.publish(vis_right_line);

			// rviz interface
			geometry_msgs::PoseStamped obs_left, obs_right;
			std::vector<geometry_msgs::PoseStamped> obs_vec = tc_->getObsDetectionArea(pos_x, pos_y, pos_th);
			obs_vec[0].header.frame_id = global_frame_;
			obs_vec[1].header.frame_id = global_frame_;
			obs_vec[2].header.frame_id = global_frame_;
			tf_->transformPose("odom", obs_vec[1], obs_left);
			tf_->transformPose("odom", obs_vec[2], obs_right);
			visualization_msgs::Marker obs_left_line, obs_right_line;
			getLineVisMsgs(pos_x, pos_y, obs_left, obs_left_line, VisAreaType::OBSDETECTAREA);
			getLineVisMsgs(pos_x, pos_y, obs_right, obs_right_line, VisAreaType::OBSDETECTAREA);
			
			test_obs_detect_left_pub_.publish(obs_left_line);
			test_obs_detect_right_pub_.publish(obs_right_line);
			
			if(!tc_->isFollowBlocked(goal_x, goal_y, pos_x, pos_y, pos_th)){
				return false;
			}
			return true;
		}
		return false;
	}

	bool DynamicPlannerROS::isStopAvoidance(){
		if (! isInitialized()){
			ROS_ERROR("base_local_planner", "This planner has not been initialized");
			return false;
		}
		tf::Stamped<tf::Pose> global_pose;
		if(!costmap_ros_->getRobotPose(global_pose)){
			return false;
		}

		// get current position of robot
		double pos_x = global_pose.getOrigin().getX();
		double pos_y = global_pose.getOrigin().getY();
		double pos_th = tf::getYaw(global_pose.getRotation());

		if(tc_->isFrontPathFree(pos_x, pos_y, pos_th)){
			return true;
		}
		return false;
	}

	bool DynamicPlannerROS::computeVelCommandsInStateOfAvoid(geometry_msgs::Twist& cmd_vel, geometry_msgs::PoseStamped& next_goal, unsigned int& following_wp_num){
		if (! isInitialized()) {
			ROS_ERROR("base_local_planner", "This planner has not been initialized");
			return false;
		}
		tf::Stamped<tf::Pose> global_pose;
		if(!costmap_ros_->getRobotPose(global_pose)){
			return false;
		}

		// get current position of robot
		double pos_x = global_pose.getOrigin().getX();
		double pos_y = global_pose.getOrigin().getY();
		double pos_th = tf::getYaw(global_pose.getRotation());

		next_goal.header.frame_id = global_frame_;
		next_goal.header.stamp = ros::Time::now();

		if(tc_->pointFollowWithTPMethod(pos_x, pos_y, pos_th, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, next_goal.pose.position.x, next_goal.pose.position.y)){
			ROS_INFO_STREAM("base_local_planner: pos (" << pos_x << "," << pos_y << ") " << "avoid [" << cmd_vel.linear.x << "," << cmd_vel.linear.y << "," << cmd_vel.angular.z << "]");
			following_wp_num = 1;
		}
		else{
			return false;
		}
		return true;
	}

	bool DynamicPlannerROS::computeVelCommandsInStateOfFollow(geometry_msgs::Twist& cmd_vel, geometry_msgs::PoseStamped& next_goal, unsigned int& following_wp_num){
		if (! isInitialized()) {
			ROS_ERROR("base_local_planner", "This planner has not been initialized");
			return false;
		}
		tf::Stamped<tf::Pose> global_pose;
		if(!costmap_ros_->getRobotPose(global_pose)){
			return false;
		}

		std::vector<geometry_msgs::PoseStamped> transformed_plan;
		if(!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)){
			ROS_WARN("base_local_planner", "Could not transform the global plan to the frame of the controller");
			return false;
		}

		//if the global plan passed in is empty... we won't do anything
		if(transformed_plan.empty())
			return false;
		tf::Stamped<tf::Pose> goal_point;
	
		if(following_wp_num < transformed_plan.size()){
			tf::poseStampedMsgToTF(transformed_plan[following_wp_num], goal_point);

			//we assume the global goal is the last point in the global plan
			double goal_x = goal_point.getOrigin().getX();
			double goal_y = goal_point.getOrigin().getY();
			double yaw = tf::getYaw(goal_point.getRotation());
			double goal_th = yaw;

			ROS_INFO_STREAM("base_local_planner: follow point (" << goal_x << "," << goal_y << ")");
	
			next_goal.header.frame_id = global_frame_;
			next_goal.header.stamp = ros::Time::now();
			next_goal.pose.position.x = goal_x;
			next_goal.pose.position.y = goal_y;

			// test
			std_msgs::String goal_test_msgs;
			std::stringstream str_goal_test;
			str_goal_test << goal_x << "," << goal_y << "," << goal_th;
			goal_test_msgs.data = str_goal_test.str();
			// test_goal_.publish(goal_test_msgs);

			double pos_x = global_pose.getOrigin().getX();
			double pos_y = global_pose.getOrigin().getY();
			double pos_th = tf::getYaw(global_pose.getRotation());

			// test
			std_msgs::String pose_test_msgs;
			std::stringstream str_pose_test;
			str_pose_test << pos_x << "," << pos_y << "," << pos_th;
			pose_test_msgs.data = str_pose_test.str();
			// test_robot_pos_.publish(pose_test_msgs);

			// test
			std_msgs::String num;
			std::stringstream str_num;
			str_num << following_wp_num;
			num.data = str_num.str();
			// test_wp_index_.publish(num);

			if(!is_usv_mode_){
				if(tc_->pointFollow(goal_x, goal_y, goal_th, pos_x, pos_y, pos_th, cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z))
					++following_wp_num;

				ROS_INFO_STREAM("base_local_planner: pos (" << pos_x << "," << pos_y << ") " << "follow [" << cmd_vel.linear.x << "," << cmd_vel.linear.y << "," << cmd_vel.angular.z << "]");
			}
			else{
				if(std::fabs(pos_x - goal_x) <= 0.05 && std::fabs(pos_y - goal_y) <= 0.05){
					++following_wp_num;
				}
			}

			// if(following_wp_num == transformed_plan.size() - 1){
			if(following_wp_num == transformed_plan.size()){
				reached_goal_ = true;
				following_wp_num = 1;
			}

			return true;
			}
		return false;
	}

	bool DynamicPlannerROS::isGoalReached() {
		if (! isInitialized()) {
			ROS_ERROR("base_local_planner", "This planner has not been initialized");
			return false;
		}
		//return flag set in controller
		return reached_goal_;
	}

  // rvis test interface
	void DynamicPlannerROS::getLineVisMsgs(double sp_x, double sp_y, double ep_x, double ep_y, visualization_msgs::Marker& line_msgs, VisAreaType vis_area_type){
		// fill data
		line_msgs.header.frame_id = "map";
		line_msgs.header.stamp = ros::Time::now();
		line_msgs.ns = "lines";
		line_msgs.action = visualization_msgs::Marker::ADD;
		line_msgs.type = visualization_msgs::Marker::LINE_LIST;
		line_msgs.scale.x = 0.01;
		
		if(vis_area_type == VisAreaType::VISUALAREA){
			line_msgs.color.b = 1.0;
		}
		else{
			line_msgs.color.r = 1.0;
		}
		line_msgs.color.a = 1.0;
		geometry_msgs::Point s_p;
		s_p.x = sp_x;
		s_p.y = sp_y;
		s_p.z = 0;
		geometry_msgs::Point e_p;
		e_p.x = ep_x;
		e_p.y = ep_y;
		e_p.z = 0;
		line_msgs.points.push_back(s_p);
		line_msgs.points.push_back(e_p);
	}

	// rviz interface
	void DynamicPlannerROS::getLineVisMsgs(double sp_x, double sp_y, geometry_msgs::PoseStamped e_pos, visualization_msgs::Marker& line_msgs, VisAreaType vis_area_type){
		double ep_x = e_pos.pose.position.x;
		double ep_y = e_pos.pose.position.y;
		getLineVisMsgs(sp_x, sp_y, ep_x, ep_y, line_msgs, vis_area_type);
	}

}; // end of namespace


