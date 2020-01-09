/**
  *****************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  dynamic_planner.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/1/9
  * - Brief:     Clean local planner used TP method
  *****************************************************************************
*/
#ifndef DYNAMIC_PLANNER_H_
#define DYNAMIC_PLANNER_H_

#include <vector>
#include <cmath>

// for obstacle data access
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <base_local_planner/footprint_helper.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/trajectory.h>
#include <base_local_planner/Position2DInt.h>
#include <base_local_planner/BaseLocalPlannerConfig.h>

// for poses of path
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

// for some datatypes transformation
#include <tf/transform_datatypes.h>

// for creating a local cost grid
#include <base_local_planner/map_cell.h>
#include <base_local_planner/map_grid.h>

// for rviz visualization
#include <visualization_msgs/Marker.h>

#define PI 3.1415926

namespace base_local_planner{
	/**
 	* @brief Enum class of void choice 
 	*/
	enum class AvoidOrientation{
    		MID,
    		LEFT,
    		RIGHT
	};

	/**
         * @class DynamicPlanner
         * @brief Computes control velocity or local target point for a robot given a costmap, a plan, and the robot's position in the world. 
         */
	class DynamicPlanner{
	public:
		/**
		 * @brief Constructs a trajectory controller
  		 * @param costmap A reference to the Costmap the controller should use
		 * @param footprint_spec A polygon representing the footprint of the robot. (Must be convex)
       		 * @param follow_vel Velocity of vehicle when follow with a path
        	 * @param ang_for_left Left of three path in state of avoidance
       	  	 * @param ang_for_right Right of three path in state of avoidance
       		 * @param obs_detect_dist Distance of obstacle detecting through visual three path
       		 * @param vis_ang_for_left Left of three path in state of dynamic obstacle checking
       		 * @param vis_ang_for_right Right of three path in state of dynamic obstacle checking 
		 */
		DynamicPlanner(WorldModel& world_model, 
			const costmap_2d::Costmap2D& costmap,
			std::vector<geometry_msgs::Point> footprint_spec,
                        double pdist_scale = 0.6, double gdist_scale = 0.8, double occdist_scale = 0.2,
			double follow_vel = 0.5, double ang_for_left = 45,
			double ang_for_right = 45, double obs_detect_dist = 1.0,
			double vis_ang_for_left = 5.0,
			double vis_ang_for_right = 5.0,
			double xy_goal_tolerance = 0.05
		);

		/**
       	         * @brief  Destructs a local planner
       		 */
		~DynamicPlanner(){};

		/**
		 * @brief Reconfigures the trajectory planner
		 */
		void reconfigure(BaseLocalPlannerConfig &cfg);

	        /**
       		 * @brief  Update the plan that the controller is following
       		 * @param new_plan A new plan for the controller to follow 
       		 * @param compute_dists Wheter or not to compute path/goal distances when a plan is updated
       		 */
      	        void updatePlan(const std::vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists = false);

      		/**
       		 * @brief  Accessor for the goal the robot is currently pursuing in world corrdinates
       		 * @param x Will be set to the x position of the local goal 
       	 	 * @param y Will be set to the y position of the local goal 
       		 */
      		void getLocalGoal(double& x, double& y);
	        
		/**
	         * @brief Compute the components and total cost for a map grid cell
	         * @param cx The x coordinate of the cell in the map grid
	         * @param cy The y coordinate of the cell in the map grid
	         * @param path_cost Will be set to the path distance component of the cost function
	         * @param goal_cost Will be set to the goal distance component of the cost function
	         * @param occ_cost Will be set to the costmap value of the cell
	         * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
	         * @return True if the cell is traversible and therefore a legal location for the robot to move to
	         */
	        bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

	        /** 
	         * @brief compute line cost of three path
	         * @param cur_pos_x x coordinate of current position
	         * @param cur_pos_y y coordinate of current position
	         * @param cur_pos_th heading of current position in radian
	         * @param line_cost_of_mid computed line cost of mid way
	         * @param line_cost_of_left computed line cost of left way
	         * @param line_cost_of_right computed line cost of right way
	         */
	        void outputCostOfThreePath(double cur_pos_x, double cur_pos_y, double cur_pos_th, double& line_cost_of_mid, double& line_cost_of_left, double& line_cost_of_right);
	        /**
	         * @brief check whether the way to follow is blocked by obstacle
	         * @param next_wp_x x coordinate of next waypoint
	         * @param next_wp_y y coordinate of next waypoint
	         * @param cur_pos_x x coordinate of current position
	         * @param cur_pos_y y coordinate of current position
	         * @param cur_pos_th orientation of current position
	         * @return True represents the way to follow is not blocked, otherwise return false
	         */
	        bool isFollowBlocked(double next_wp_x, double next_wp_y, double cur_pos_x, double cur_pos_y, double cur_pos_th );

	        /**
	         * @brief check whether the three paths are free with obstacle
	         * @param cur_pos_x x coordinate of current position
	         * @param cur_pos_y y coordinate of current position
	         * @param cur_pos_th heading of current position in radian
	         * @return True represnets three paths are free, otherwise return false
	         */
	        bool isFrontPathFree(double cur_pos_x, double cur_pos_y, double cur_pos_th);


	        /**
	         * @brief follow point with TP method
	         * @param cur_pos_x x coordinate of current position
	         * @param cur_pos_y y coordinate of current position
	         * @param cur_pos_th orientation of current position
	         * @param vx variety to store compute result of x velocity
	         * @param vy variety to store compute result of y velocity
	         * @param vth variety to store compute result of angular velocity
	         * @param goal_x X coordinate of goal
	         * @param goal_y Y coordinate of goal
	         * @return is there any error in computation
	         */
	        bool pointFollowWithTPMethod(double cur_pos_x, double cur_pos_y, double cur_pos_th, double& vx, double& vy, double& vth, double& goal_x, double& goal_y);

	        /**
	         * @brief  LOS point following logical
	         * @param  different between target pose and current orientation
	         * @param  vx x velocity
	         * @param  vy y velocity
	         * @param  vtheta angular velocity 
	         * @return 
	         */
	        void pointFollowLOS(double theta_det, double& vx, double& vy, double& vth);

	        /**
	         * @brief LOS point following algorithm
	         * @param goal_x x of target 
	         * @param goal_y y of target
	         * @param goal_th orientation of target
	         * @param x_i x of robot
	         * @param y_i y of robot
	         * @param th_i orientation of robot
	         * @param vx x linear velocity
	         * @param vy y linear velocity
	         * @param vth angular velocity
	         * @return whether reached the goal
	         */
	        bool pointFollow(double& goal_x, double& goal_y, double& goal_th, double& x_i, double& y_i, double& th_i, double& vx, double& vy, double& vth);

	        /**
	         * @brief Ooutput parameters of visual area in pose stampe
	         * @param cur_pos_x current x coordinate of robot in world frame
	         * @param cur_pos_y current y coordinate of robot in world frame
	         * @param cur_pos_th current orientation of robot in world frame
	         * @return Visual area parameters in pose stampe
	         */
	        std::vector<geometry_msgs::PoseStamped> getVisualArea(double cur_pos_x, double cur_pos_y, double cur_pos_th);


	        /**
	         * @brief Ooutput parameters of obstacle detecting area in pose stampe
	         * @param cur_pos_x current x coordinate of robot in world frame
	         * @param cur_pos_y current y coordinate of robot in world frame
	         * @param cur_pos_th current orientation of robot in world frame
	         * @return Obstacle detecting area parameters in pose stampe
	         */
	        std::vector<geometry_msgs::PoseStamped> getObsDetectionArea(double cur_pos_x, double cur_pos_y, double cur_pos_th);

	        /**
	         * @brief Output parameters of visual area
	         * @param cur_pos_x current x coordinate of vehicle
	         * @param cur_pos_y current y coordinate of vehicle
	         * @param cur_pos_th current orientation of vehicle
	         * @param mid_x x coordinate in world frame of mid way
	         * @param mid_y y coordinate in world frame of mid way
	         * @param vis_left_x x coordinate in world frame of visual left way
	         * @param vis_lett_y y coordinate in world frame of visual left way
	         * @param vis_right_x x coordinate in world frame of visual right way
	         * @param vis_right_y y coordinate in world frame of visual right way
	         */
	        void getVisualArea(double cur_pos_x, double cur_pos_y, double cur_pos_th, double& mid_x, double& mid_y, double& vis_left_x, double& vis_left_y, double& vis_right_x, double& vis_right_y);

	        /**
	         * @brief Output parameters of obstacle detectiing area
	         * @param cur_pos_x current x coordinate of vehicle
	         * @param cur_pos_y current y coordinate of vehicle
	         * @param cur_pos_th current orientation of vehicle
	         * @param mid_x x coordinate in world frame of mid way
	         * @param mid_y y coordinate in world frame of mid way
	         * @param left_x x coordinate in world frame of obstacle detecting left way
	         * @param lett_y y coordinate in world frame of obstacle detecting left way
	         * @param right_x x coordinate in world frame of obstacle detecting right way
	         * @param right_y y coordinate in world frame of obstacle detecting right way
	         */
	        void getObsDetectionArea(double cur_pos_x, double cur_pos_y, double cur_pos_th, double& mid_x, double& mid_y, double& left_x, double& left_y, double& right_x, double& right_y);

	        /** @brief Return the footprint specification of the robot. */
	        geometry_msgs::Polygon getFootprintPolygon() const { return costmap_2d::toPolygon(footprint_spec_); }
	        std::vector<geometry_msgs::Point> getFootprint() const { return footprint_spec_; }
	
	private:
	        /**
	         * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
	         * @param x_i The x position of the robot 
	         * @param y_i The y position of the robot 
	         * @param theta_i The orientation of the robot
	         * @return 
	         */
	        double footprintCost(double x_i, double y_i, double theta_i);
	
		/**
		 * @brief  Compute x position based on velocity
		 * @param  xi The current x position
		 * @param  vx The current x velocity
		 * @param  vy The current y velocity
		 * @param  theta The current orientation
		 * @param  dt The timestep to take
		 * @return The new x position 
		*/
		inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt){
			return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
		}

		/**
		 * @brief  Compute y position based on velocity
		 * @param  yi The current y position
		 * @param  vx The current x velocity
		 * @param  vy The current y velocity
		 * @param  theta The current orientation
		 * @param  dt The timestep to take
		 * @return The new y position 
		*/
		inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt){
			return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
		}

		double lineCost(int x0, int x1, int y0, int y1, bool mode = false);
		
		double pointCost(int x, int y);
	
	private:
		base_local_planner::FootprintHelper footprint_helper_;

        	MapGrid path_map_; ///< @brief The local map grid where we propagate path distance
        	MapGrid goal_map_; ///< @brief The local map grid where we propagate goal distance

		const costmap_2d::Costmap2D& costmap_; ///< @brief Provides access to cost map information

        	WorldModel& world_model_; ///< @brief The world model that the controller uses for collision detection

        	std::vector<geometry_msgs::Point> footprint_spec_; ///< @brief The footprint specification of the robot

        	std::vector<geometry_msgs::PoseStamped> global_plan_; ///< @brief The global path for the robot to follow
		
		double goal_x_,goal_y_; ///< @brief Storage for the local goal the robot is pursuing

		double final_goal_x_, final_goal_y_; ///< @brief The end position of the plan.		
		
 		bool final_goal_position_valid_; ///< @brief True if final_goal_x_ and final_goal_y_ have valid data.  Only false if an empty path is sent.

		double inscribed_radius_, circumscribed_radius_;

		double gdist_scale_, pdist_scale_, occdist_scale_; ///< @brief Scaling factors for the controller's cost function

		double follow_interrupt_dist_; // interrupt distance to determine whether robot has reached the gaol

		double kp_, ki_, kd_; // pid parameter

		double theta_det_last_;
		double theta_det_add_;

		double obs_detect_dist_;
		double ang_for_left_;
		double ang_for_right_;
		double follow_vel_;

		double vis_ang_for_left_;
		double vis_ang_for_right_;

		unsigned int avoid_count_;
		AvoidOrientation last_choice_;
		boost::mutex configuration_mutex_;

	};
};// end of namespalce


#endif
