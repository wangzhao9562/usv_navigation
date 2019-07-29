Base_local_planner
局部规划器功能包

Add path following realization based on LOS algorithm:
增加基于LOS算法的路经跟随实现:

1.Add some parameter used in LOS logic in .cfg and class TrajectoryPlanner
  在.cfg文件和类型TrajectroyPlanner中增加了LOS逻辑相关的配置参数
  
2.The basic LOS logic is realized in class TrajectoryPlanner
  在TrajectoryPlanner中增加了实现LOS逻辑的方法

3.Interface between move_base and LOS logic is realized in class TrajectoryPlannerROS
  在TrajectroyPlannerROS中增加了move_base与LOS逻辑之间的接口