static_planner
静态规划器功能包

1.static_planner is equal to global_planner in original navigation pkg, used to get a global plan for robot between start and goal
  静态规划器等同于原导航包的全局规划器, 用于为机器人规划当前位置到目标位置的全局路径

2.static_planner use advanced astar which is developed for USVs and other similar under-actuated mobile robot, the planned path is not just a reference for robot, however, can be approximatively followed by mobile robot.
  静态规划器采用改进A*算法, 用于USVs或其他相似的欠驱动移动式机器人的路径规划。该改进A*算法不仅是作为机器人的参考路径, 而是可以完全近似近似的被机器人跟随。
  
Advanced A*:
1.The heuristic function in common A* is: fn = gn + hn, which represents the whole cost for a mobile robot or vehicle move from the start to the target position. In this formula, the gn describes the cost between start to the searched position, this part is always known in searching process, therefore, anthor part just representsthe cost from the searched position to the target, however, this part is unknown for us. 
![](https://github.com/wangzhao9562/usv_navigation/blob/master/static_planner/PICTURE/formular.png)
![](https://github.com/wangzhao9562/usv_navigation/blob/master/static_planner/PICTURE/common.png)

2.Usually, we use Euclidean or Manhattan distance represents hn, therefore, A* algorithm has the property to get the nearly distance-shortest path between two pointsin map. However, whatever using Manhattan or Euclidean distance, the result is only available for mass point model to follow wtih. Consider the simplest condition, theshortest path between two points is the straight line when this line doesn't cross any obstacle, however, when the orientation of robot is reverse to the path(the heading of path defined as orientation from start to target), this path cannot be followed with if the robot is not a holonomic one.
![](https://github.com/wangzhao9562/usv_navigation/blob/master/static_planner/PICTURE/prob1.png)
![](https://github.com/wangzhao9562/usv_navigation/blob/master/static_planner/PICTURE/prob2.png)

3.Consider the performance of the limit of orientation, and we want to keep the heuristic property of A* algorithm at the same time, we can define the cost compensate for different orientation of robot in different situations and add this compensate into gn. From this way, when we compare current fn value and recoreded fn value of one node in each searching process, both of the orientation compensate and heuristic part can affect the result of comparison.
![](https://github.com/wangzhao9562/usv_navigation/blob/master/static_planner/PICTURE/pic1.png)
![](https://github.com/wangzhao9562/usv_navigation/blob/master/static_planner/PICTURE/pic2.png) 