## Workspace Visualization
====
It is relatively very easy to visualize the map created in map_creator package. Please run rviz first in a terminal.

rosrun rviz rviz

Go to add new display, add the display panel of your desired map (reachability/ capability) and set the topic to /reachability_map or /capability_map
In a second terminal load the map (load any one map at a single time, otherwise it may lock up the system) 
To load reachability map:

rosrun map_creator load_reachability_map motorman_mh5_r0.08_rechability.h5

To load capability map:

rosrun map_creator load_capability_map motorman_mh5_r0.08_capability.h5

You can also visualize your inverse reachability map, by the same process as reachability map. Just provide the name of your inverse reachability map name instead of reachability map.
The map would look like mostly a sphere with color variation based on reachability index. Though the inverse reachability map has an uneven shape as it is the transformation of the reachability map.
The blue spheres are the most reachable upto red for less reachability. The red spheres are mostly located at the outer space of the workspace. The size of the spheres can be altered by changing the values in the display. The default shape is sphere. It can also be changed to cone, cylinder and boxes. The workspace can be cut in halves from the display panel.  The reachable poses can also be visualized by checking the checkbox in the display.
For a more detailed description of the features of workspace_visualization display panel, please refer to [http://wiki.ros.org/reuleaux] (http://wiki.ros.org/reuleaux)
