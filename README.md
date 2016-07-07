# reuleaux
GSoC Project for robot reachability

Please read this file before using this package

***This package is constantly updated and still in progress. So please use with caution ***

(This package depends on the ikfast package. So please modify and put your own robots ikfast.cpp file location in the kinematics.h file in the include folder before start building)

__map_creator__ 

This package creates two types of maps. 1. Reachability Map, 2. Capability Map
Both have their own functionalities. To create a reahability map,

rosrun map_creator create_reachability_map

the default robot for now is motoman_mh5 and the resolution is 0.035. It will create a new folder called maps in the map_creator package and there you can find a databse file mentioning the robot name and resolution.
For creating the capability map, the process is almost the same. You have to run

rosrun map_creator create_capability_map

It will also create a new database file mentioning the robot name and resolution

__workspace_visualization__

this package visualizes the reachability map and capability map in rviz. If everything goes right you can have new display panels in rviz with the tab of reachability map and capability map. Select whihch one you would like to visualize. (Dont run both of them at the same time, otherwise the system may hang up)

rosrun workspace_visualization load_reachability_map <map location><map_name>  
rosrun workspace_visualization load_capability_map <map location><map_name> 

e.g

rosrun workspace_visualization load_reachability_map /home/abhi/Desktop/motoman_mh5_r0.035_reachability.h5

choose the appropriate topic and you should see the maps.

