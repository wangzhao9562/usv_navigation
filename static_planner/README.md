static_planner
静态规划器功能包

1.static_planner is equal to global_planner in original navigation pkg, used to get a global plan for robot between start and goal
  静态规划器等同于原导航包的全局规划器, 用于为机器人规划当前位置到目标位置的全局路径

2.static_planner use advanced astar which is developed for USVs and other similar under-actuated mobile robot, the planned path is not just a reference for robot, however, can be approximatively followed by mobile robot.
  静态规划器采用改进A*算法, 用于USVs或其他相似的欠驱动移动式机器人的路径规划。该改进A*算法不仅是作为机器人的参考路径, 而是可以完全近似近似的被机器人跟随。
  
Advanced A*:
1.The heuristic function in common A* is: fn = gn + hn, which represents the whole cost for a mobile robot or vehicle move from the start to the target position. In this formula, the gn describes the cost between start to the searched position, this part is always known in searching process, therefore, anthor part just representsthe cost from the searched position to the target, however, this part is unknown for us. 
  经典A*算法的启发式采用fn = gn + hn, fn代表了移动式机器人从起始位置到目标位置走过的全局路径代价。其中, gn表示搜索过程中从起始节点到当前搜索节点记录的代价, fn表示剩余的从当前搜索节点到目标节点的代价, 但这一部分在搜索过程中是始终未知的(除非采用双端同时搜索)
![](https://github.com/wangzhao9562/usv_navigation/blob/master/static_planner/PICTURE/formular.png)
![](https://github.com/wangzhao9562/usv_navigation/blob/master/static_planner/PICTURE/common.png)

2.Usually, we use Euclidean or Manhattan distance represents hn, therefore, A* algorithm has the property to get the nearly distance-shortest path between two points in map. However, whatever using Manhattan or Euclidean distance, the result is only available for mass point model to be followed wtih. Consider the simplest condition, the shortest path between two points is the straight line when this line doesn't cross any obstacle, however, when the orientation of robot is reverse to the path(the heading of path defined as orientation from start to target), this path cannot be completely followed with if the robot is not a holonomic one. Otherwise, if there is any obstacle cross the practical trajectory of the mobile robot and no sensors and dynamic obstacle avoiding ability are used, this will be unsafe for the robot. 
  通常, 我们采用欧几里得或曼哈顿距离代表hn的值, 因此, A*算法具备得到近似最短路径的性质。但无论采用欧几里得或曼哈顿距离, 得到的路径只适用于质点模型跟随。考虑最简单的情况：假设起始位置与目标位置之间没有任何障碍物, 那么此时两点间最短的路径应该是直线段, 但当此时要跟随路径的机器人航向与路径方向相反(规定路径方向从起始位置指向目标位置), 那么这条路径就不能被非全驱动机器人完整的跟随。此外, 当有障碍物存在于机器人实际路径上时, 若机器人不具备环境感知和动态避障能力, 还存在安全隐患。
![](https://github.com/wangzhao9562/usv_navigation/blob/master/static_planner/PICTURE/prob1.png)
![](https://github.com/wangzhao9562/usv_navigation/blob/master/static_planner/PICTURE/prob2.png)

3.Consider the performance of the limit of orientation, and we want to keep the heuristic property of A* algorithm at the same time, we can define the cost compensate for different orientation of robot in different situations and add this compensate into gn. From this way, when we compare current fn value and recoreded fn value of one node in each searching process, both of the orientation compensate and heuristic part can affect the result of comparison.
  考虑航向限制产生的影响, 同时使算法具备使规划的路径代价尽量短的性质, 我们可以定义搜索过程中航向影响的代价补偿并将其加入到gn值中。这样在搜索过程中, 每当比较一个节点当前的fn值和记录的fn值时, 航向代价补偿部分和启发式部分都能对结果产生影响。
![](https://github.com/wangzhao9562/usv_navigation/blob/master/static_planner/PICTURE/pic1.png)
![](https://github.com/wangzhao9562/usv_navigation/blob/master/static_planner/PICTURE/pic2.png) 