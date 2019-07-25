static_planner
静态规划器

1.static_planner is equal to global_planner in original navigation pkg, used to get a global plan for robot between start and goal
  静态规划器等同于原导航包的全局规划器, 用于为机器人规划当前位置到目标位置的全局路径

2.static_planner use advanced astar which is developed for USVs and other similar under-actuated mobile robot, the planned path 
is not just a reference for robot, however, can be approximatively followed by mobile robot.
  静态规划器采用改进A*算法, 用于USVs或其他相似的欠驱动移动式机器人的路径规划。该改进A*算法不仅是作为机器人的参考路径, 而是可以完全近似近似
  的被机器人跟随。
  
