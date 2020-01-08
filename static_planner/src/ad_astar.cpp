#include<static_planner/ad_astar.h>
#include<costmap_2d/cost_values.h>
#include<string>
#include<fstream>
#include<ios>
#include<math.h>

namespace static_planner {

AdAStarPlanner::AdAStarPlanner(int xs, int ys, double resolution) :
        Planner(xs, ys), end_index_(NULL), resolution_(resolution)
{
  
}

bool AdAStarPlanner::getPlan(unsigned char* costs, double start_x, double start_y, double start_th, double end_x, double end_y, double end_th, int cycles, std::vector< std::pair<float, float> >& plan)
{
        double start_x_up = std::ceil(start_x), start_y_up = std::ceil(start_y);
        double end_x_up = std::ceil(end_x), end_y_up = std::ceil(end_y);

	wrapper_.initialize(costs, toIndex(start_x_up, start_y_up), lethal_cost_, nx_, ny_, rough_lenth_, resolution_, unknown_);
        // wrapper_.printNewCosts();
        // wrapper_.printOriCosts();
	
	std::pair<int, int> start_wrapper = wrapper_.getCoordInNewCostmap(toIndex(start_x_up, start_y_up));
	double start_x_wrapper = start_wrapper.first;
	double start_y_wrapper = start_wrapper.second;
	
	std::pair<int, int> end_wrapper = wrapper_.getCoordInNewCostmap(toIndex(end_x_up, end_y_up));
	double end_x_wrapper = end_wrapper.first;
	double end_y_wrapper = end_wrapper.second;
	
	int* new_costs = wrapper_.getNewCosts();
	
	// if(searchPath(costs, start_x, start_y, start_th, end_x, end_y, end_th, cycles)){
	if(searchPath(new_costs, start_x_wrapper, start_y_wrapper, start_th, end_x_wrapper, end_y_wrapper, end_th, wrapper_.getNewMapWidth() * wrapper_.getNewMapHeight() * 2)){
            // ROS_INFO("try to get path from goal recursively");
       
	    Index* temp = end_index_.p_;
            plan.push_back(std::pair<float, float>(end_x, end_y));
            
	    while(temp->p_ != nullptr){
		// int x = temp->i_ % nx_, y = temp->i_ / nx_;
		int ns = wrapper_.getIndexInCostmap(temp->i_);
		int x = ns % nx_, y = ns / nx_;
                // ROS_INFO("push (%d, %d) into path", x, y);
	    	plan.push_back(std::pair<float, float>((float)x, (float)y));
                
                temp = temp->p_;
	    }
            plan.push_back(std::pair<float, float>(start_x, start_y)); // add start point
	    return true;
	}
	else
	    return false;
}


bool AdAStarPlanner::searchPath(int* costs, double start_x, double start_y, double start_th, double end_x, double end_y, double end_th, int cycles){
    // ROS_INFO("start to search path");	
	
    total_distance_ = hypot(end_x - start_x, end_y - start_y);										
    queue_.clear();

    // Cal start position
    int start_th_f = orientationFilter(start_th); // get start th after filtering
    std::pair<int, int> next_pos = nextPos(start_th_f); // get next Index along the orientation
    int start_i = wrapper_.toIndex(static_cast<int>(start_x) + next_pos.first, static_cast<int>(start_y) + next_pos.second);
    // check if next pos is reachable
    if(costs[start_i] == 1){
      ROS_ERROR("static_planner: invalid start input");
      return false;
    }
    queue_.push_back(Index(start_i, 0, start_th_f));

    // Cal goal position 
    int end_th_f = orientationFilter(end_th);
    std::pair<int, int> previous_pos = previousPos(end_th_f); 
    // int goal_i = wrapper_.toIndex(end_x, end_y);
    int goal_i = wrapper_.toIndex(static_cast<int>(end_x) + previous_pos.first, static_cast<int>(end_y) + previous_pos.second);
    // check if last pos is reachable
    if(costs[goal_i] == 1){
      ROS_ERROR("static_planner: invalid goal input");
      return false;
    }    

    // initialize counter
    int cycle = 0;

    int new_nx_ = wrapper_.getNewMapWidth();

    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1()); // move minimum value to the end of container

        int i = top.i_;
        if (i == goal_i){
	   end_index_ = top;
           return true;
	}
		
	queue_.pop_back(); // pop the Index with minimum cost
		
	// go through all neighbor of Index with minimum cost
        traverse(costs, top, i, (int)end_x, (int)end_y);     

        cycle++;
    }

    return false;
}

void AdAStarPlanner::traverse(int* costs, Index min_fn_ind, int i, int end_x, int end_y){
    int new_nx = wrapper_.getNewMapWidth();
    int min_fn_ind_th = min_fn_ind.th_;
    if(min_fn_ind_th == 0){
        add(costs, min_fn_ind, i, i + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx + 1, end_x, end_y);
    }
    if(min_fn_ind_th == 45){
        add(costs, min_fn_ind, i, i + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx, end_x, end_y);
        // add(costs, min_fn_ind, i, i + new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx + 1, end_x, end_y);
    }
    if(min_fn_ind_th == 90){
        add(costs, min_fn_ind, i, i + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx + 1, end_x, end_y);
    }
    if(min_fn_ind_th == 135){
        add(costs, min_fn_ind, i, i + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx + 1, end_x, end_y);
    }
    if(min_fn_ind_th == -45){
        add(costs, min_fn_ind, i, i + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx + 1, end_x, end_y);
    }
    if(min_fn_ind_th == -90){
        add(costs, min_fn_ind, i, i + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx + 1, end_x, end_y);
    }
    if(min_fn_ind_th == -135){
        add(costs, min_fn_ind, i, i + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx - 1, end_x, end_y);
    }
    if(min_fn_ind_th == 180 || min_fn_ind_th == -180){
        add(costs, min_fn_ind, i, i - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i + new_nx + 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx - 1, end_x, end_y);
        add(costs, min_fn_ind, i, i - new_nx + 1, end_x, end_y);
    }

}

void AdAStarPlanner::add(int* costs, Index min_fn_ind, int i, int next_i, int end_x,int end_y) {
    int new_nx_ = wrapper_.getNewMapWidth();

    if (next_i < 0 || next_i >= wrapper_.getNewMapSize()){
        return;
    }
	
	if(costs[next_i] == 1)
		return;
    
    int next_ind_th = CalcIndexOrientation(next_i - i, new_nx_);
	int det_th = abs(min_fn_ind.th_ - next_ind_th); // calc the delta th between min_fn_ind and neighbor ind
	if(det_th > 180)
		det_th = 360 - det_th;
	
	float compensate = CalcCompensate(min_fn_ind.th_, det_th);
	
    int x = next_i % new_nx_, y = next_i / new_nx_;
    // float distance = abs(end_x - x) + abs(end_y - y); // manhattan distance 
	float distance = hypot(end_x - x, end_y - y); // euclidean distance
	
	int pos_in_list = isInList(next_i);
	
	if(pos_in_list >= 0){
		// float gn = calcG(min_fn_ind, i, next_i - i);
	        float gn = calcG(min_fn_ind, i, next_i - i) + compensate;	 
                // float hn = calcH(compensate, distance);
		float hn = distance;
                float fn = gn + hn;
		if(fn < queue_[pos_in_list].fn_){
			// ROS_INFO("change property of neighbor");
                        queue_[pos_in_list].gn_ = gn;
			queue_[pos_in_list].fn_ = fn;
			queue_[pos_in_list].p_= new Index(min_fn_ind.i_, min_fn_ind.fn_, min_fn_ind.gn_, min_fn_ind.th_, min_fn_ind.p_);
			queue_[pos_in_list].th_ = next_ind_th;
		} 
	}
	else{
		// ROS_INFO("neighbor is not in list, add it into list");
		// float gn = calcG(min_fn_ind, i, next_i - i);
	        float gn = calcG(min_fn_ind, i, next_i - i) + compensate;	 
          	// float hn = calcH(compensate, distance);
		float hn = distance;
                float fn = gn + hn;
		queue_.push_back(Index(next_i, fn, gn, next_ind_th, new Index(min_fn_ind.i_, min_fn_ind.fn_, min_fn_ind.gn_, min_fn_ind.th_, min_fn_ind.p_)));
		std::make_heap(queue_.begin(), queue_.end(), greater1());
	}

    // queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    // std::push_heap(queue_.begin(), queue_.end(), greater1()); // construct minimum heap
}


float AdAStarPlanner::calcG(Index cur_ind, int ind, int det_ind)const{
	float extra_g = abs(det_ind) == 1 ?  u_cost1_ : u_cost2_;
	return extra_g + cur_ind.gn_; 
}

float AdAStarPlanner::calcH(float compensate, float distance){
	// return (compensate + distance / total_distance_) * 10 * neutral_cost_;
        // return (compensate + distance / total_distance_);
        return (compensate + distance / total_distance_);
}

int AdAStarPlanner::isInList(int ind){
	for(int i = 0; i < queue_.size(); i++){
		if(queue_[i].i_ == ind){
			return i;
		}
	}
	return -1;
}
int AdAStarPlanner::orientationFilter(double th){
    int th_filter;
    if(th > -0.392699 && th < 0.392699){
        th_filter = 0;
    }
    else if(th >= 0.392699 && th < 1.178097){ 
        th_filter = 45;
    }
    else if(th >= 1.178097 && th < 1.963496) //[67.5, 112.5) -> 90
    {
        th_filter = 90;
    }
    else if(th >= 1.963496 && th < 2.748894) //[112.5, 157.5) -> 135
    {
        th_filter = 135;
    }
    else if(th >= 2.748894 && th <= 3.141593) //[157.5, 180] -> 180
    {
        th_filter = 180;
    }
    else if(th > -1.178097 && th <= -0.392699) //(-67.5, 22.5] -> -45
    {
        th_filter = -45;
    }
    else if(th > -1.963496 && th <= -1.178097) //(-112.5, 67.5] -> -90
    {
        th_filter = -90;
    }
    else if(th > -2.748894 && th <= -1.963496) //(-157.5, -112.5] -> -135
    {
        th_filter = -135;
    }
    else if(th >= -3.1415 && th <= -2.748894) //[-180, 157.5] -> -180
    {
        th_filter = -180;
    }
	
	return th_filter;
}

std::pair<int, int> AdAStarPlanner::previousPos(int cur_th_filter){
   std::pair<int, int> det_pos;
   if(cur_th_filter == 135){
     det_pos.first = 1;
     det_pos.second = -1;
   }
   if(cur_th_filter == -135){
     det_pos.first = 1;
     det_pos.second = 1;
     return det_pos;
   }
   if(cur_th_filter == -90){
     det_pos.first = 0;
     det_pos.second = 1;
     return det_pos;
   }
   if(cur_th_filter == 45){
     det_pos.first = -1;
     det_pos.second = -1;
     return det_pos;
   }
   if(cur_th_filter == -45){
     det_pos.first = -1;
     det_pos.second = 1;
     return det_pos;
   }
   if(cur_th_filter == 0){
     det_pos.first = -1;
     det_pos.second = 0;
     return det_pos;
   }
   if(cur_th_filter == 90){
     det_pos.first = 0;
     det_pos.second = -1;
     return det_pos;
   }
   if(cur_th_filter == 180){
     det_pos.first = 1;
     det_pos.second = 0;
     return det_pos;
   }
   if(cur_th_filter == -180){
     det_pos.first = 1;
     det_pos.second = 0;
     return det_pos;
   }
	
   det_pos.first = 0;
   det_pos.second = 0;
	
   return det_pos;
}

std::pair<int, int> AdAStarPlanner::nextPos(int cur_th_filter){
	std::pair<int, int> det_pos;
	if(cur_th_filter == 135){
		det_pos.first = -1;
		det_pos.second = 1;
		return det_pos;
	}
	if(cur_th_filter == -135){
		det_pos.first = -1;
		det_pos.second = -1;
		return det_pos;
	}
	if(cur_th_filter == -90){
		det_pos.first = 0;
		det_pos.second = -1;
		return det_pos;
	}
	if(cur_th_filter == 45){
		det_pos.first = 1;
		det_pos.second = 1;
		return det_pos;
	}
	if(cur_th_filter == -45){
		det_pos.first = 1;
		det_pos.second = -1;
		return det_pos;
	}
	if(cur_th_filter == 0){
		det_pos.first = 1;
		det_pos.second = 0;
		return det_pos;
	}
	if(cur_th_filter == 90){
		det_pos.first = 0;
		det_pos.second = 1;
		return det_pos;
	}
	if(cur_th_filter == 180){
		det_pos.first = -1;
		det_pos.second = 0;
		return det_pos;
	}
	if(cur_th_filter == -180){
		det_pos.first = -1;
		det_pos.second = 0;
		return det_pos;
	}
	
	det_pos.first = 0;
	det_pos.second = 0;
	
	return det_pos;
}

int AdAStarPlanner::CalcIndexOrientation(int det_ind, int nx){
	int orientation;
	if(det_ind == -nx - 1){
		orientation = 135;
		return orientation;
	}
	if(det_ind == nx - 1){
		orientation = -135;
		return orientation;
	}
	if(det_ind == nx){
		orientation = -90;
		return orientation;
	}
	if(det_ind == -nx + 1){
		orientation = 45;
		return orientation;
	}
	if(det_ind == nx + 1){
		orientation = -45;
		return orientation;
	}
	if(det_ind == 1){
		orientation = 0;
		return orientation;
	}
	if(det_ind == -nx){
		orientation = 90;
		return orientation;
	}
	if(det_ind == -1){
		orientation = 180;
		return orientation;
	}
	return 0; // default go straight
}

float AdAStarPlanner::CalcCompensate(float cur_th, float th_det){
	float cost_compensate;
	
	if(cur_th == 0 || cur_th == 90 || cur_th == 180 || cur_th == -90){
		if(th_det == 0){
			cost_compensate = 0;
			return cost_compensate;
		}
		if(th_det == 45){
			cost_compensate = 2;
			return cost_compensate;
		}
		if(th_det == 90){
			cost_compensate = 3;
			return cost_compensate;
		}
		if(th_det == 135){
			cost_compensate = 4;
			return cost_compensate;
		}
		if(th_det == 180){
			cost_compensate = 3 + sqrt(2);
			return cost_compensate;
		}
	}
	else{
		if(th_det == 0){
			cost_compensate = 0;
			return cost_compensate;
		}
		if(th_det == 45){
			cost_compensate = sqrt(2);
			return cost_compensate;
		}
		if(th_det == 90){
			cost_compensate = 2 * sqrt(2);
			return cost_compensate;
		}
		if(th_det == 135){
			cost_compensate = 2 + sqrt(2);
			return cost_compensate;
		}
		if(th_det == 180){
			cost_compensate = 3 + sqrt(2);
			return cost_compensate;
		}
	}
	return 1e6;
}

} //end namespace static_planner
#include<static_planner/ad_astar.h>
#include<costmap_2d/cost_values.h>

#include<static_planner/ad_astar.h>
#include<costmap_2d/cost_values.h>
#include<static_planner/ad_astar.h>
#include<costmap_2d/cost_values.h>
#include<static_planner/ad_astar.h>
#include<costmap_2d/cost_values.h>
