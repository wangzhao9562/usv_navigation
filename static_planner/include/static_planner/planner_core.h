#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H

#define POT_HIGH 1.0e10        // unassigned cell potential
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>

#include <static_planner/planner.h>
#include <static_planner/orientation_filter.h>
#include <static_planner/StaticPlannerConfig.h>

namespace static_planner {

class Planner;
// class GridPath;

/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the global_planner planner which runs a fast, interpolated navigation function on a costmap.
 */

class StaticPlanner : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief  Default constructor for the StaticPlanner object
         */
        StaticPlanner();

        /**
         * @brief  Constructor for the StaticPlanner object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  frame_id Frame of the costmap
         */
        StaticPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief  Default deconstructor for the StaticPlanner object
         */
        ~StaticPlanner();

        /**
         * @brief  Initialization function for the StaticPlanner object
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance The tolerance on the goal point for the planner
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                      std::vector<geometry_msgs::PoseStamped>& plan);
					  
        /**
         * @brief  Publish a path for visualization purposes
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

    protected:

        /**
         * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
         */
        costmap_2d::Costmap2D* costmap_;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        bool initialized_, allow_unknown_;

    private:
        void mapToWorld(double mx, double my, double& wx, double& wy);
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);
        // void publishPotential(float* potential);
		bool getPlan(const geometry_msgs::PoseStamped& goal, std::vector< std::pair<float, float> >& path, std::vector<geometry_msgs::PoseStamped>& plan);
		void reconfigureCB(static_planner::StaticPlannerConfig &config, uint32_t level);
		void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
		
        double planner_window_x_, planner_window_y_, default_tolerance_;
        std::string tf_prefix_;
        boost::mutex mutex_;
        ros::ServiceServer make_plan_srv_;

        // PotentialCalculator* p_calc_;
        Planner* planner_;
        // Traceback* path_maker_;
        OrientationFilter* orientation_filter_;

        // bool publish_potential_;
        // ros::Publisher potential_pub_;
        // int publish_scale_;
		
        unsigned char* cost_array_;
        // float* potential_array_;
        unsigned int start_x_, start_y_, end_x_, end_y_;

        bool old_navfn_behavior_;
		bool use_orientation_filter_;
		
        float convert_offset_;

        dynamic_reconfigure::Server<static_planner::StaticPlannerConfig> *dsrv_;

};

} //end namespace static_planner

#endif
