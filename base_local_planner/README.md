Base_local_planner

Add path following realization based on LOS algorithm:
1.Add some parameter used in LOS logic in .cfg and class TrajectoryPlanner
2.The basic LOS logic is realized in class TrajectoryPlanner
3.Interface between move_base and LOS logic is realized in class TrajectoryPlannerROS