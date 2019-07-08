move_base

Modified control bus in:
1. MoveBase::excuteCb
2. MoveBase::planThread
3. MoveBase::excuteCycle

After modified, logic of control bus is: 

WAITING -> PLANNING/REPLANNING #Manual
PLANNING/REPLANNING -> FOLLOWING #After planning, robot follows the path
FOLLOWING -> CONTROLLING #If local map does not match with global map, switch into dynamic planning
CONTROLLING -> REPLANNING #After dynamic planning, replan path to goal
FOLLOWING -> WAITING #Reach the goal
WAITING -> CONTROLLING #Local map dose not match with global map and no mission is in execute