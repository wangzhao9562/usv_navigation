#include <static_planner/costmap_wrapper.h>
#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <string>
#include <exception>

namespace static_planner{
	
	CostmapWrapper::CostmapWrapper(unsigned char* costs, int position_index, int lethal_cost, int map_w, int map_h, double rough_len_world, double resolution, bool unknown){
		costs_ = costs;
		new_costs_ = nullptr;
		position_index_ = position_index;
		lethal_cost_ = lethal_cost;
		map_width_ = map_w;
		map_height_ = map_h;
		rough_len_world_ = rough_len_world;
		resolution_ = resolution;
		is_initialized_ = false;

                unknown_ = unknown;
	
		rough_len_map_ = static_cast<int>(std::ceil(rough_len_world_ / resolution_));
               
                test_last_index_ = -1;

                cnt_cost_1_ = 0;
                cnt_cost_0_ = 0;
	}
	
	void CostmapWrapper::initialize(){
		getVertexIndex();
		getNewCostsOfNewCostmap();
                recordWindowCost();
		is_initialized_ = true;
	}
	
	void CostmapWrapper::initialize(unsigned char* costs, int position_index, int lethal_cost, int map_w, int map_h, double rough_len_world, double resolution, bool unknown){
		if(!is_initialized_){
			costs_ = costs;
		    new_costs_ = nullptr;
		    position_index_ = position_index;
		    lethal_cost_ = lethal_cost;
		    map_width_ = map_w;
		    map_height_ = map_h;
		    rough_len_world_ = rough_len_world;
		    resolution_ = resolution;

                    unknown_ = unknown;
	        
		    rough_len_map_ = static_cast<int>(std::ceil(rough_len_world_ / resolution_));
		    
                    test_last_index_ = -1; 
            
                    cnt_cost_1_ = 0;
                    cnt_cost_0_ = 0;
 
                    ROS_INFO("start wrapper initialization");        
 		    initialize();
		}
                else if(position_index != position_index_){
                    if(new_costs_ != nullptr){
                         ROS_INFO("delete new costmap costs");
                         delete new_costs_;
                         new_costs_ = nullptr;
                    }

                    costs_ = costs;
                    position_index_ = position_index;
                    lethal_cost_ = lethal_cost;
                    map_width_ = map_w;
                    map_height_ = map_h;
                    rough_len_world_ = rough_len_world;
                    resolution_ = resolution;

                    unknown_ = unknown;

                    rough_len_map_ = static_cast<int>(std::ceil(rough_len_world_ / resolution_));

                    test_last_index_ = -1;
                    cnt_cost_1_ = 0;
                    cnt_cost_0_ = 0;

                    ROS_INFO("restart wrapper initialization");
                    initialize();
                }
	}
	
	CostmapWrapper::~CostmapWrapper(){
		if(new_costs_ != nullptr)
			delete new_costs_;
	}

	
	int CostmapWrapper::getIndexInNewCostmap(int index){
		if(is_initialized_){
			int rough_factor = 2 * rough_len_map_;
                        int det_x = index % map_width_ - lowerleft_x_;
                        int det_y = index / map_width_ - lowerleft_y_;
			// int det_x = (index - lowerleft_index_) % map_width_;
		    // int det_y = (index - lowerleft_index_) / map_width_;
		    return det_x / rough_factor + (det_y / rough_factor) * new_map_width_;
		}
		else
			return -1;
	}
	
	std::pair<int, int> CostmapWrapper::getCoordInNewCostmap(int index){
		std::pair<int, int> pos;
		if(is_initialized_){
			int rough_factor = 2 * rough_len_map_;
                        int det_x = index % map_width_ - lowerleft_x_;
                        int det_y = index / map_width_ - lowerleft_y_;
			// int det_x = (index - lowerleft_index_) % map_width_ ;
		        // int det_y = (index - lowerleft_index_) / map_width_ ;
			pos.first = det_x / rough_factor;
			pos.second = det_y / rough_factor;
		    return pos;
		}
		else{
			pos.first = -1;
			pos.second = -1;
			return pos;
		}
	}
	
	
	
	int CostmapWrapper::getIndexInCostmap(int index){
		if(is_initialized_){
			if(project_table_.find(index) != project_table_.end()){
				return project_table_[index];
			}
			return -1;
		}
		return -1;
	}
	
	void CostmapWrapper::getVertexIndex(){
                ROS_INFO("calc size of new costmap");
		int x_ll = position_index_ % map_width_, y_ll = position_index_ / map_width_;
		int x_ur = x_ll, y_ur = y_ll;
		int cntx_ll = 0, cnty_ll = 0, cntx_ur = 0, cnty_ur = 0;
		
		int rough_factor = rough_len_map_ * 2;
		
		while(x_ll > rough_factor){
			x_ll -= rough_factor;
			cntx_ll++;
		}
		while(y_ll > rough_factor){
			y_ll -= rough_factor;
			cnty_ll++;
		}
		
		while(x_ur < map_width_ - rough_factor){
			x_ur += rough_factor;
			cntx_ur++;
		}
		while(y_ur < map_height_ - rough_factor){
			y_ur += rough_factor;
			cnty_ur++;
		}
		
		new_map_width_ = cntx_ll + cntx_ur;
		new_map_height_ = cnty_ll + cnty_ur;

                lowerleft_x_ = x_ll;
                lowerleft_y_ = y_ll;
		
		position_index_new_ = cntx_ll + cnty_ll * new_map_width_;
		lowerleft_index_ = x_ll + y_ll * map_width_;
        
        }

	void CostmapWrapper::getNewCostsOfNewCostmap(){
                try{
		    new_costs_ = new int[new_map_width_ * new_map_height_];
                }
                catch(std::exception& e){
                    ROS_WARN("wrong to build new costmap");
                }
		int rough_factor = 2 * rough_len_map_;
		
		for(int x = 0; x < new_map_width_; x++){
			for(int y = 0; y < new_map_height_; y++){
                                int x_in_costs = lowerleft_x_ + rough_factor * x;
                                int y_in_costs = lowerleft_y_ + rough_factor * y;
				// int index_in_costs = lowerleft_index_ + rough_factor * x + rough_factor * y * map_width_;
			        int index_in_costs = x_in_costs + y_in_costs * map_width_;
              
                                int index_in_new_costs = x + y * new_map_width_;
				new_costs_[index_in_new_costs] = getCostOfWindow(x_in_costs, y_in_costs);
			    project_table_[index_in_new_costs] = index_in_costs;
			}
		}
	}
	
	int CostmapWrapper::getCostOfWindow(int x, int y){
		// int rough_factor = 2 * rough_len_map_;
		
		// int index_lowerleft = index - rough_factor - rough_factor * map_width_;
		// int index_upperright = index + rough_factor + rough_factor * map_width_;

                // if(test_last_index_ == index)
                    //  ROS_INFO("equal to last index");

                // test_last_index_ = index;	      
     
	        // int index_lowerleft = index - rough_len_map_ - rough_len_map_ * map_width_;
		// int index_upperright = index + rough_len_map_ + rough_len_map_ * map_width_;
           
                // int x = index % map_width_;
                // int y = index / map_width_;		

		// for(int ind = std::max(index_lowerleft, 0); ind <= std::min(index_upperright, map_width_ * map_height_ - 1); ind++){
		// for(int x = index_lowerleft % new_map_width_; x <= index_upperright % new_map_width_; x++)
                   // for(int y = index_lowerleft / new_map_width_; y <= index_upperright / new_map_width_; y++){
                       // if(transformCost(toIndex(x, y)) == 1){
				// return 1;
		       // }
		// }

                for(int i = -rough_len_map_; i <= rough_len_map_; i++)
                   for(int j = -rough_len_map_; j <= rough_len_map_; j++){
                      int ind_sub = x + i + (y + j) * map_width_;
                      if(transformCost(ind_sub) == 1)
                          return 1;
                }
		
		return 0;
	}
	
	int CostmapWrapper::transformCost(int index){
	    if(static_cast<int>(costs_[index]) >= lethal_cost_ && !(unknown_ && costs_[index] == costmap_2d::NO_INFORMATION)){
                cnt_cost_1_++;
	    	return 1;
	    }

            if(static_cast<int>(costs_[index]) != costmap_2d::LETHAL_OBSTACLE){
                cnt_cost_0_++;
	        return 0;
            }
            
            cnt_cost_1_++;
            return 1;
	}


        void CostmapWrapper::printOriCosts()const{
            std::string filename = "/home/wz9562/Documents/ori_costs_rec.txt";
            std::fstream s;
            s.open(filename);
            if(!s.is_open()){
               ROS_INFO("wrong to open ori cost record");
            }
            else{
               unsigned char* ptr = costs_;
               int i = 0;
               while(i < map_height_ * map_width_){
                 s << static_cast<int>(*ptr) << " ";
                 ptr++;
                 i++;
               }
               s << i;
            }
            s.close();
        }


        void CostmapWrapper::printNewCosts()const{
            std::string filename = "/home/wz9562/Documents/new_costs_rec.txt";
            std::fstream s(filename, s.binary | s.trunc | s.in | s.out);
            if(!s.is_open()){
               ROS_INFO("wrong to open cost record");
            }
            else{
               int i = 0;
               int* ptr = new_costs_;
               while(i < new_map_height_ * new_map_width_){
                  s << *ptr << " ";
                  ptr++;  
                  i++; 
               }
               s << i;
            }
            s.close();                                   
        }

        void CostmapWrapper::recordWindowCost()const{
            std::string filename = "/home/wz9562/Documents/cost_cnt_rec.txt";
            std::fstream s(filename, s.binary | s.trunc | s.in | s.out);
            if(!s.is_open()){
               ROS_INFO("wrong to open file");
            }
            else{
               s << cnt_cost_1_ << " " << cnt_cost_0_;
            }
            s.close();
        }
	
}
