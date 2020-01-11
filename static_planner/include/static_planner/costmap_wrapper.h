#ifndef _COSTMAP_WRAPPER_H
#define _COSTMAP_WRAPPER_H
 
#include<map>
#include<string>
#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<costmap_2d/cost_values.h>
#include<costmap_2d/costmap_2d.h>

namespace static_planner{
	
class CostmapWrapper{
	public:
	    CostmapWrapper() : global_map_(nullptr), new_costs_(nullptr){ 
		is_initialized_ = false; 
	    };
	
	    /* 
         * @breif Construct function, compute radius of robot in costmap
         * @Param costs cost array of costmap
         * @Param position_index position of robot in costmap
         * @Param lethal_cost value of lethal_cost in costmap
         * @Param map_w width(x) of costmap
         * @Param map_h height(y) of costmap
         * @Param r_in_world radius of robot in world
         * @Param resolution resolution of costmap
         */
		CostmapWrapper(costmap_2d::Costmap2D* costmap, std::string global_frame, int position_index, int lethal_cost, int map_w, int map_h, double rough_len_world, double resolution, bool unknown);

		~CostmapWrapper();
		
		void initialize(costmap_2d::Costmap2D* costmap, std::string global_frame, int position_index, int lethal_cost, int map_w, int map_h, double rough_len_world, double resolution, bool unknown);
		
		void initialize();
  	
		int* getNewCosts()const{
			if(is_initialized_)
				return new_costs_;
		}
		
		nav_msgs::OccupancyGrid getOccupancyGrid(){
			return grid_;
		}
		
		int getNewMapSize()const{
			return new_map_height_ * new_map_width_;
		}
		
		int toIndex(int x, int y)const{
			return x + y * new_map_width_;
		}

                int getNewMapWidth()const{
                        return new_map_width_;
                }

                int getNewMapHeight()const{
                        return new_map_height_;
                }

                int getRoughLen()const{
                        return rough_len_map_;
                }
	
                double getRoughLenInWorld()const{
                        return rough_len_world_;  
                }	

                double getOriMapResolution()const{
                        return resolution_;
                }

                int getLowerLeft()const{
                        return lowerleft_index_;
                }

                int getLowerLeftX()const{
                        return lowerleft_x_;
                }
   
                int getLowerLeftY()const{
                        return lowerleft_y_;
                }

                void printNewCosts()const;

                void printOriCosts()const;

                void recordWindowCost()const;

	    /* 
         * @breif transform input index of costsmap to indexs of new costmap
         * @Param index input index
         * @return index in new costmap
         */	
        int getIndexInNewCostmap(int index);
		
		/* 
         * @breif transform input index of new costmap to index of costmap, only for waypoints of planned path
         * @Param index input index
         * @return index in costmap
         */	
		int getIndexInCostmap(int index);
		
		/* 
         * @breif transform input index of new costmap to index of costmap, only for waypoints of planned path
         * @Param index input index
         * @return coordination in costmap
         */	
	std::pair<int, int> getCoordInNewCostmap(int index);
	
	private:
	    /* 
         * @breif go through points in window of given point, return cost of window
         * @Param costs cost of center point in cost array
         * @return cost of window
         */
	    // int getCostOfWindow(int index);
	 int getCostOfWindow(int x, int y);	
		/* 
         * @breif transform cost in costmap into 0 or 1
         * @Param index index of point
         * @return 0 or 1
         */	
	    int transformCost(int index);
		
		/* 
         * @breif get lowerleft Index of new costmap in original costs array, and index of robot position in new costmap 
         */	
		void getVertexIndex();
		
		/* 
         * @breif get lowerleft Index of new costmap in original costs array, and index of robot position in new costmap 
         */	
		void getNewCostsOfNewCostmap();
	
	    void getWrapperGrid();
	
	    /* members */
	    unsigned char* costs_;
		int position_index_;
		int lethal_cost_;
		int map_width_;
		int map_height_;
		double rough_len_world_;
		int rough_len_map_;
		double resolution_;
		
		bool is_initialized_;
		
		int new_map_width_;
		int new_map_height_;
		int lowerleft_index_;
		int position_index_new_;
	        std::map<int, int> project_table_;
		int* new_costs_;

                bool unknown_;

                int test_last_index_;

                int lowerleft_x_;
                int lowerleft_y_;

		double origin_x_; // coordinate in world frame
		double origin_y_;

                int cnt_cost_1_;
                int cnt_cost_0_;

		nav_msgs::OccupancyGrid grid_; // rviz visualization
		costmap_2d::Costmap2D* global_map_;
		
		std::string global_frame_;
};
		
}; // end of namespace


#endif
