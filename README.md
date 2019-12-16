usv_navigation

Modified navigation pkg for USVs and under-actuated AUVs based on ros navigation stack

Modified the global base planner interface for advanced A* which considers the orientation limitation of under-actuated mobile robot and this navigation stack is available without kinetic model.

Origin navigation stack: https://github.com/ros-planning/navigation

Address of simulation package: https://github.com/wangzhao9562/my_nav_test

Simulation with turtlebot: 
Find responding ros package from https://github.com/turtlebot and compile it in catkin space  
or sudo apt-get install ros-ros_version-turtlebot*

Notice: 
1. For using existed simulation tools and third-part libraries, name of package is not modifed to satiesfy with the demand of interface,  
please don't mix this package with origin navigation stack in one catkin space.
2. Programe is developed based on origin navigation stack in version of kinetic devel, the version of system should be ubuntu 16.04LTS better.

Screenshot of simulation 
![](https://github.com/wangzhao9562/usv_navigation/blob/master/assets/screenshot_for_nav_pub.png)  
![](https://github.com/wangzhao9562/usv_navigation/blob/master/assets/screenshot_for_nav_pub_02.png)  
![](https://github.com/wangzhao9562/usv_navigation/blob/master/assets/screenshot_for_nav_pub_03.png)  
![](https://github.com/wangzhao9562/usv_navigation/blob/master/assets/screenshot_for_nav_pub_04.png)