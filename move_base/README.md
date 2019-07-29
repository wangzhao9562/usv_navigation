move_base
控制总线功能包

Modified control bus in:
相比原导航栈在以下几处有改动:

1. MoveBase::excuteCb
2. MoveBase::planThread
3. MoveBase::excuteCycle

After modified, logic of control bus is: 
经改动后, 控制总线的逻辑为：

WAITING -> PLANNING/REPLANNING #Manual, for example, give a target to robot in standby
待机状态 -> 规划状态/重规划状态 #手动改变, 如在待机状态下输入目标点

PLANNING/REPLANNING -> FOLLOWING #After planning, robot follows the path
规划状态/重规划状态 -> 跟随状态  #规划完成后, 机器人开始跟随路径

FOLLOWING -> CONTROLLING #If local map does not match with global map, switch into dynamic planning
跟随状态 -> 控制状态 #当局部地图与全局地图不匹配时, 进入控制状态进行动态避障

CONTROLLING -> REPLANNING #After dynamic planning, replan path to goal
控制状态 -> 重规划状态 #动态避障后, 重规划到目标的路径

FOLLOWING -> WAITING #Reach the goal
跟随状态 -> 待机状态 #到达目标点后, 待机

WAITING -> CONTROLLING #Local map dose not match with global map and no mission is in execute
待机状态 -> 控制状态 #在待机状态下局部地图与全局地图不匹配, 进入控制状态进行动态避障