/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  dynamic_planner_ros.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/1/9
  * - Brief:     ROS wrapper class of clean local planner used TP method
  *****************************************************************************
*/

#ifndef DYNAMIC_PLANNER_ROS_H_
#define DYNAMIC_PLANNER_ROS_H_

#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/point_grid.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/voxel_grid_model.h>
#include <base_local_planner/trajectory_planner.h>
#include <base_local_planner/map_grid_visualizer.h>

#include <base_local_planner/planar_laser_scan.h>

#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <tf/transform_listener.h>

#include <boost/thread.hpp>

#include <string>

#include <angles/angles.h>

#include <nav_core/base_local_planner.h>

#include <dynamic_reconfigure/server.h>
#include <base_local_planner/BaseLocalPlannerConfig.h>

#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/dynamic_planner.h>

// rviz test interface
#include <visualization_msgs/Marker.h>

namespace base_local_planner{
	/**
	 * @class DynamicPlannerROS
	 * @brief A ROS wrapper for the trajectory controller that queries the param server to construct a controller
	 */
	class DynamicPlannerROS : public nav_core::BaseLocalPlanner{
	public:	
		/**
	 	 * @brief  Default constructor for the ros wrapper
	 	 */
		DynamicPlannerROS();

		/**
		 * @brief  Constructs the ros wrapper
		 * @param name The name to give this instance of the trajectory planner
		 * @param tf A pointer to a transform listener
		 * @param costmap The cost map to use for assigning costs to trajectories
		 */
		DynamicPlannerROS(std::string name, 
			tf::TransformListener* tf, 
			costmap_2d::Costmap2DROS* costmap_ros);

		/**
		 * @brief  Constructs the ros wrapper
		 * @param name The name to give this instance of the trajectory planner
		 * @param tf A pointer to a transform listener
		 * @param costmap The cost map to use for assigning costs to trajectories
		 */
		void initialize(std::string name, 
			tf::TransformListener* tf, 
			costmap_2d::Costmap2DROS* costmap_ros);

		/**
		 * @brief  Destructor for the wrapper
		 */
		~DynamicPlannerROS();
		
		/**
		 * @brief whether the way to follow with is blocked by obstacle
		 * @param following_wp_num index of way point in followed planned path
		 * @return True represents the followed way is blocked, otherwise returns false
		 */
		bool isFollowBlocked(unsigned int following_wp_num);

		/**
		 * @brief whether the robot can quit from state of avoidance
		 * @return True represents robot can switch state of avoiding to planning
		 */
		bool isStopAvoidance();

		/**
		 * @brief Compute velocity command in state of avoiding
		 * @param cmd_vel Used to store computed velocity command
		 * @return Ture if a valid command was computed, false otherwise
		 */
		bool computeVelCommandsInStateOfAvoid(geometry_msgs::Twist& cmd_vel, geometry_msgs::PoseStamped& goal, unsigned int& following_wp_num);

		/**
		 * @brief  Given the current position, orientation, and velocity of the robot,
		 * compute velocity commands to send to the base 
		 * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
		 * @return True if a valid trajectory was found, false otherwise
		 */
		bool computeVelCommandsInStateOfFollow(geometry_msgs::Twist& cmd_vel, geometry_msgs::PoseStamped& goal, unsigned int& following_wp_num);

		/**
		 * @brief  Set the plan that the controller is following
		 * @param orig_global_plan The plan to pass to the controller
		 * @return True if the plan was updated successfully, false otherwise
		 */
		bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

		/**
		 * @brief  Check if the goal pose has been achieved
		 * @return True if achieved, false otherwise
		 */
		bool isGoalReached();

		bool isInitialized() {
			return initialized_;
		}

		/** @brief Return the inner DynamicPlanner object.  Only valid after initialize(). */
		DynamicPlanner* getPlanner() const { return tc_; }

	private:
		/**
		* @brief type for visualization test 
		*/
		enum class VisAreaType{
			VISUALAREA,
			OBSDETECTAREA
		};


		/**
		 * @brief Callback to update the local planner's parameters based on dynamic reconfigure
		 */
		void reconfigureCB(BaseLocalPlannerConfig &config, uint32_t level);

		double sign(double x){
			return x < 0.0 ? -1.0 : 1.0;
		}

		// rviz test interface
		void getLineVisMsgs(double sp_x, double sp_y, double ep_x, double ep_y, visualization_msgs::Marker& line_msgs, VisAreaType vis_area_type);

		void getLineVisMsgs(double sp_x, double sp_y,  geometry_msgs::PoseStamped e_pos, visualization_msgs::Marker& line_msgs, VisAreaType vis_area_type);

	private:
		WorldModel* world_model_; ///< @brief The world model that the controller will use
		DynamicPlanner* tc_; ///< @brief The trajectory controller

		costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
		costmap_2d::Costmap2D* costmap_; ///< @brief The costmap the controller will use
		MapGridVisualizer map_viz_; ///< @brief The map grid visualizer for outputting the potential field generated by the cost function
		tf::TransformListener* tf_; ///< @brief Used for transforming point clouds
		std::string global_frame_; ///< @brief The frame in which the controller will run
		double max_sensor_range_; ///< @brief Keep track of the effective maximum range of our sensors
		nav_msgs::Odometry base_odom_; ///< @brief Used to get the velocity of the robot
		std::string robot_base_frame_; ///< @brief Used as the base frame id of the robot
		double xy_goal_tolerance_, yaw_goal_tolerance_;

		std::vector<geometry_msgs::PoseStamped> global_plan_;
		boost::recursive_mutex odom_lock_;

		bool reached_goal_;

		ros::Publisher g_plan_pub_, l_plan_pub_;

		bool initialized_;
		base_local_planner::OdometryHelperRos odom_helper_;

		dynamic_reconfigure::Server<BaseLocalPlannerConfig> *dsrv_;
		base_local_planner::BaseLocalPlannerConfig default_config_;
		bool setup_;


		std::vector<geometry_msgs::Point> footprint_spec_;

		// test
		ros::Publisher test_wp_index_;
		ros::Publisher test_goal_;
		ros::Publisher test_robot_pos_;

		ros::Publisher test_mid_pub_;
		ros::Publisher test_vis_left_pub_;
		ros::Publisher test_vis_right_pub_;
		ros::Publisher test_obs_detect_left_pub_;
		ros::Publisher test_obs_detect_right_pub_;

		bool is_usv_mode_;
	}; // end of class
}; // end of namespace

#endif
