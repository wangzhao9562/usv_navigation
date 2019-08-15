usv_navigation

Modified navigation pkg for USVs and under-actuated AUVs based on ros navigation stack
基于ros导航栈改动的导航功能包，用于USVs和欠驱动AUVs的自主导航

The global path will not be just a reference for mobile robot, however, robot should try to follow the whole path which is planned by global planner.
全局规划路径不仅是作为移动机器人的参考路径, 移动机器人要尝试跟随由全局规划器规划的全局路径

Modified the global base planner interface for advanced A* which considers the orientation limitation of under-actuated mobile robot
为实现考虑欠驱动移动机器人航向约束的A*算法修改了基础全局规划器接口

Origin navigation stack: https://github.com/ros-planning/navigation
原导航功能包地址: https://github.com/ros-planning/navigation

Switching between following mode and controlling mode has not been tested
跟随模式和控制模式的自动切换尚未测试

Simulation with rbx1 robot: https://github.com/pirobot/rbx1
![](https://github.com/wangzhao9562/usv_navigation/blob/master/assets/simulation01.png)
![](https://github.com/wangzhao9562/usv_navigation/blob/master/assets/simulation02.png)
![](https://github.com/wangzhao9562/usv_navigation/blob/master/assets/simulation03.png)