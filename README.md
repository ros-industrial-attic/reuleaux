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

rosrun map_creator load_reachability_map <map location><map_name>  
rosrun map_creator load_capability_map <map location><map_name> 

e.g

rosrun map_creator load_reachability_map /home/abhi/Desktop/motoman_mh5_r0.035_reachability.h5

choose the appropriate topic and you should see the maps.



__Base Placement__

The interface for base placement plugin is in progress.....
To visualize the base placement (for now) please go through these steps:

1. Creating the inverse reachability map

rosrun map_creator create_inverse_reachability_map <reachability map name>

rosrun map_creator create_inverse_reachability motoman_mh5_0.08_reachability.h5

it will create an inverse reachability map file from a reachability map defined in the previous steps. The next steps are needed just for now. All will be done in the plugin interface.

2. Creating the Base Placement map

The grasp poses are defined in the file. Please change the grasp co-ordinates according to need. This map will be created based on the inverse reachability map created in the previous step.

rosrun map_creator create_base_place_map motoman_mh5_0.08_Inv_reachability.h5

It will now create an workspace map (union of transformed map from all the grasp poses) with spheres and arrows and saves a file of base_place.h5 It can be easily visualized by the workspace visualization panel and setting the topic to /reachability_map

3. Creating final base poses

There are mainly four methods to get the base (Final decision is pending)

First in a terminal run:

rosrun map_creator show_base

You have to run this file, otherwise the base placement process will not start. This will visualize the final base poses as magenta arrows. Then in a different terminal

rosrun map_creator show_grasp_pose

It will show the grasp poses as green arrows.

running rviz add two interactive_markers. Set the topics as

/base_poses
/grasp_poses

For running them, the process is same for all the methods:

rosrun map_creator find_base_byAverage motoman_mh5_0.08_Base_place.h5

rosrun map_creator find_base_byPCA motoman_mh5_0.08_Base_place.h5

rosrun map_creator find_base_byGraspReachabilityScore motoman_mh5_0.08_Base_place.h5

rosrun map_creator find_base_byIKSolutionScore motoman_mh5_0.08_Base_place.h5


The parameter numofFinalPose decides how many base poses are visualized 

The parameter numofSp defines from how many spheres we are collecting all the poses for base placement criterion (ByGraspReachabilityScore and ByIKSolutionScore)

In the process terminal it will also show the reachability of each new base pose with every grasp pose.




















