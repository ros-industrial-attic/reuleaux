## Base Placement plugin
====
Please refer to [ros wiki] (http://wiki.ros.org/reuleaux) for more detailed instruction

#1. RViz and RQT User Interface:

There are two types of interactive markers:
  - The red arrow acts as a pointer which the user can move around the RViz enviroment. Fruthermore by clicking on the arrow another magenta arrow is added to the RViz enviroment. This arrow acts as task poses for base placement planner.
  - The magenta arrow is the task poses for the base placement planner. The orientation of the arrow can be changed by holding the CTRL key and moving it with the mouse.
  - Each arrow has a menu where the user can either delete the selected arrow or it can change its position and orientation by using the 6DOF marker control.
  - The RQT UI communicates simultaniously with the RViz enviroment and the User can change the state of a marker either through RViz or the RQT UI 
  - TreeView displays all the added waypoints. The user can manipulate them directly in the TreeView and see their position and orientation of each waypoint.
  - The user can add new point or delete it through the RQT UI.
  - New tool component has been added for adding Arrows by using a mouse click

#2. How to run the code
  Just run rosrun rviz rviz

  - From the panel window select base placement planner
  - Add a reachability map display and set the topic to /reachability_map
  - Add an Interactive Marker display and set the topic to /base_placement_plugin/update
  - Add a markerArray display. Set the topic to /visualization_marker_array

There is a preconfigured rviz config file in the rviz folder

After deciding the task poses load an inverse reachability map previously created. If you have not done it yet, run

rosrun map_creator create_inverse_reachability_map <name of the reachability map>

rosrun map_creator create_inverse_reachability_map /home/abhi/Desktop/motomap_mh5_r0.08_reachability.h5

it will save an invese reachability map in the map_creator/Inv_map folder

After loading the inverse reachability map set the desired parameters. There are two parameters. Number of desired base locations and number of high scoring spheres from where the poses will be collected.

Set the desired output visualization method.(The robot model visualization is still in development. Coming Soon)
```
When everything is set up, press the Find Base button. It will show the base locations.

If you want to see the union map, press the show union map button.




