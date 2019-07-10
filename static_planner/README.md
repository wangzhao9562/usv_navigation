static_planner

1.static_planner is equal to global_planner in original navigation pkg, used to get a global plan for robot between start and goal
2.static_planner use advanced astar which is developed for USVs and other similar under-actuated mobile robot, the planned path 
is not just a reference for robot, however, can be approximatively followed by mobile robot.