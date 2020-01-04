^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for usv_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Bug in v2.0 and formal(2020-1-4)
-------------------
Find a problem when planner_frequency is set as a positive number, the global planner will keep state of planning.
* Contributors: Zhao Wang(HUST)

v2.0 (2020-1-3)
-------------------
Map loader canbe an optional method to get static map for global planner in map server
Map loader receive static map from workstation like QGC through mavlink protocol
* Contributors: Zhao Wang(HUST)

