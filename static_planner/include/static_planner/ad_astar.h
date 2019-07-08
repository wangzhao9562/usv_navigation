#ifndef _AD_ASTAR_H
#define _AD_ASTAR_H

#include <static_planner/planner_core.h>
#include <static_planner/planner.h>
#include <static_planner/costmap_wrapper.h>
#include <vector>
#include <algorithm>

namespace static_planner {
class Index {
    public:
        Index(int ind) : fn_(0), gn_(0){
            i_ = ind;
	    p_ = nullptr;
        }
		
		Index(int ind, float f, float th) : gn_(0){
            i_ = ind;
            fn_ = f;
	    th_ = th;
	    p_ = nullptr;
        }
		
		Index(int ind, float f, float g, float th, Index* index){
            i_ = ind;
            fn_ = f;
			gn_ = g;
			th_ = th;
			p_ = index;
        }
		
		bool operator==(Index& index){
			if(this->i_ == index.i_)
				return true;
			return false;
		}
		
		void operator=(Index index){
			i_ = index.i_;
			fn_ = index.fn_;
			gn_ = index.gn_;
			p_ = index.p_;
			th_ = index.th_;
		}

        void operator=(Index* index){
                i_ = index->i_;
                fn_ = index->fn_;
                gn_ = index->gn_;
                p_ = index->p_;
                th_ = index->th_;
        }
		
        int i_; // index of Index in cost array
        float fn_; // fn 
		float gn_; // gn
		Index* p_; // parent Index 
		float th_; // orientation of Index
};

struct greater1 {
        bool operator()(const Index& a, const Index& b) const {
            return a.fn_ > b.fn_;
        }
};

class AdAStarPlanner : public Planner {
    public:
        AdAStarPlanner(int nx, int ny, double resolution);
		
		~AdAStarPlanner(){};
		
        // bool searchPath(unsigned char* costs, double start_x, double start_y, double start_th, double end_x, double end_y, double end_th, int cycles);
        bool getPlan(unsigned char* costs, double start_x, double start_y, double start_th, double end_x, double end_y, double end_th, int cycles,
					 std::vector< std::pair<float, float> >& plan);
    protected:
	    // bool searchPath(unsigned char* costs, double start_x, double start_y, double start_th, double end_x, double end_y, double end_th, int cycles);
		bool searchPath(int* costs, double start_x, double start_y, double start_th, double end_x, double end_y, double end_th, int cycles);
	
	private:
        void traverse(int* costs, Index ind, int i, int end_x, int end_y);  
 
        // void add(unsigned char* costs, Index ind, int i, int next_i, int end_x, int end_y);
        void add(int* costs, Index ind, int i, int next_i, int end_x, int end_y);
		
		std::pair<int, int> nextPos(int cur_th_filter); // used for advanced A*
		float calcG(Index cur_ind, int ind, int det_ind)const;
		float calcH(float compensate, float distance);
        int isInList(int ind); // used for advanced A*		
		int orientationFilter(double th);
		int CalcIndexOrientation(int det_ind, int nx);
		float CalcCompensate(float cur_th, float th_det);
		
		std::vector<Index> queue_;
		Index end_index_;
		
		const float u_cost1_ = 1.0f;
		const float u_cost2_ = 1.41f;
		float total_distance_;
		
		double resolution_;
		
		CostmapWrapper wrapper_;
};

} //end namespace static_planner
#endif

