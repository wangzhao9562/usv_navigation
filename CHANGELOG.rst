^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for usv_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

v1.0 (2020-1-9)
-------------------
Add DynamicPlanner and DynamicPlannerROS which are separated from the base local planner with dwa algorithm(TrajectoryPlanner and TrajectoryPlannerROS)
Default base local planner is changed in move_base
Common velocity command interface of base_local_planner is removed from nav_core::BaseLocalPlanner

Bug in v0.0(2020-1-4)
-------------------
Find a problem when planner_frequency is set as a positive number, the global planner will keep state of planning.
* Contributors: Zhao Wang(HUST)

v0.0 (2020-1-3)
-------------------
Map loader canbe an optional method to get static map for global planner in map server
Map loader receive static map from workstation like QGC through mavlink protocol
* Contributors: Zhao Wang(HUST)

