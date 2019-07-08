#ifndef _PLANNER_H
#define _PLANNER_H
#include <static_planner/planner_core.h>
          
namespace static_planner {

class Planner {
    public:
        Planner(int nx, int ny) :
                unknown_(true), lethal_cost_(253), neutral_cost_(50), factor_(3.0){
            setSize(nx, ny);
        }
		
		virtual bool getPlan(unsigned char* costs, double start_x, double start_y, double start_th, double end_x, double end_y, double end_th, int cycles,
			 std::vector< std::pair<float, float> >& plan) = 0;
			 
        virtual bool searchPath(int* costs, double start_x, double start_y, double start_th, double end_x, double end_y, double end_th, int cycles) = 0;

        /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        virtual void setSize(int nx, int ny) {
            nx_ = nx;
            ny_ = ny;
            ns_ = nx * ny;
        } /**< sets or resets the size of the map */
        void setLethalCost(unsigned char lethal_cost) {
            lethal_cost_ = lethal_cost;
        }
        void setNeutralCost(unsigned char neutral_cost) {
            neutral_cost_ = neutral_cost;
        }
        void setFactor(float factor) {
            factor_ = factor;
        }
        void setHasUnknown(bool unknown) {
            unknown_ = unknown;
        }
		
		void setRoughLength(double rough_lenth){
			rough_lenth_ = rough_lenth;
		}

    protected:
        inline int toIndex(int x, int y) {
            return x + nx_ * y;
        }

        int nx_, ny_, ns_; /**< size of grid, in pixels */
        bool unknown_;
        unsigned char lethal_cost_, neutral_cost_;
        int cells_visited_;
        float factor_;
		
		double rough_lenth_;
};

} //end namespace static_planner
#endif
