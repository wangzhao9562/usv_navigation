#include <static_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <static_planner/ad_astar.h>

#include <exception>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(static_planner::StaticPlanner, nav_core::BaseGlobalPlanner)

namespace static_planner{

StaticPlanner::StaticPlanner() : costmap_(NULL), initialized_(false), allow_unknown_(true){}

StaticPlanner::StaticPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) : 
                             costmap_(NULL), initialized_(false), allow_unknown_(true)
{
	initialize(name, costmap, frame_id);
}

StaticPlanner::~StaticPlanner(){
	if(planner_)
		delete planner_;
	if(dsrv_)
		delete dsrv_;
}
	
void StaticPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

void StaticPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
	initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void StaticPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id){
	if(!initialized_){
		ros::NodeHandle private_nh("~/" + name);
		costmap_ = costmap;
		frame_id_ = frame_id;
		
		unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();
		
        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
		private_nh.param("use_orientation_filter", use_orientation_filter_, false);
		
		if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;
		
		planner_ = new AdAStarPlanner(cx, cy, costmap->getResolution());
		
		orientation_filter_ = new OrientationFilter();
		
		plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
		
		private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        // private_nh.param("publish_scale", publish_scale_, 100);
		
		//get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        make_plan_srv_ = private_nh.advertiseService("make_plan", &StaticPlanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<static_planner::StaticPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<static_planner::StaticPlannerConfig>::CallbackType cb = boost::bind(
                &StaticPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
	}
	else
		ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

void StaticPlanner::reconfigureCB(static_planner::StaticPlannerConfig& config, uint32_t level) {
    planner_->setLethalCost(config.lethal_cost);
    // path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
	planner_->setRoughLength(config.rough_length);
	orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

void StaticPlanner::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool StaticPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void StaticPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool StaticPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool StaticPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) 
{
    return makePlan(start, goal, default_tolerance_, plan);
}

bool StaticPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           double tolerance, std::vector<geometry_msgs::PoseStamped>& plan) 
{
	boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }
	
	//clear the plan, just in case
	plan.clear();
	
	ros::NodeHandle n;
	std::string global_frame = frame_id_;
	
    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }
	
	if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {
		ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }
	
	double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;
        // double start_th = start.pose.orientation.z;
	// double goal_th = goal.pose.orientation.z;
	double start_roll, start_pitch, start_th;
        double goal_roll, goal_pitch, goal_th;
        tf::Quaternion start_quat, goal_quat;
        tf::quaternionMsgToTF(start.pose.orientation, start_quat);
        tf::quaternionMsgToTF(goal.pose.orientation, goal_quat);
        tf::Matrix3x3(start_quat).getRPY(start_roll, start_pitch, start_th);
        tf::Matrix3x3(goal_quat).getRPY(goal_roll, goal_pitch, goal_th);
	
	if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }
	
	if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }
	else{
        worldToMap(wx, wy, start_x, start_y);
    }
	
	wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }
	
	//clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, start_x_i, start_y_i);
	
	int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
	
	planner_->setSize(nx, ny);
	
	outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);
	
	std::vector< std::pair<float, float> > path;

        bool found_legal = false;


	try{
          found_legal = planner_->getPlan(costmap_->getCharMap(), start_x, start_y, start_th, goal_x, goal_y, goal_th, nx * ny * 2, path);
        }
        catch(std::exception& e){
          ROS_INFO("Wrong to get plan in planner");
        }

	
	if(found_legal){
		if(getPlan(goal, path, plan)){
			geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
		}
		else{
			ROS_ERROR("Failed to get a plan when a legal path was found. This shouldn't happen.");
		}
	}
	else{
		ROS_INFO("Failed to get a plan from planner");
	}
	
	// add orientations if needed
	if(use_orientation_filter_){
		orientation_filter_->processPath(start, plan);
	}

    ROS_INFO("publish plan");
    //publish the plan for visualization purposes
    publishPlan(plan);
	
    // delete potential_array_;
    return !plan.empty();
}

void StaticPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
	if(!initialized_){
		ROS_ERROR("This planner has not been initialize yet, but it is being used, please call initialize() before use");
	    return;
	}
	
	// create a message for the plan
	nav_msgs::Path gui_path;
	gui_path.poses.resize(path.size());
	
	gui_path.header.frame_id = frame_id_;
	gui_path.header.stamp = ros::Time::now();
	
	// Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool StaticPlanner::getPlan(const geometry_msgs::PoseStamped& goal, std::vector< std::pair<float, float> >& path, std::vector<geometry_msgs::PoseStamped>& plan){
	if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }
	
	std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();
	
	ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

}

