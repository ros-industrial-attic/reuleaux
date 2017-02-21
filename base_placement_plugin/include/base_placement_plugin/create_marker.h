#ifndef CREATE_MARKER_H
#define CREATE_MARKER_H

#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<visualization_msgs/InteractiveMarker.h>
#include<tf2/LinearMath/Transform.h>
#include<tf2/LinearMath/Quaternion.h>

#include<moveit/move_group_interface/move_group.h>

#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit/robot_model/robot_model.h>
#include<moveit/robot_model/joint_model_group.h>

#include<moveit/robot_state/robot_state.h>

typedef std::multimap<std::vector<double>,geometry_msgs::Pose> BasePoseJoint;

class CreateMarker{
public:
  CreateMarker(std::string group_name);


  bool makeArmMarker(BasePoseJoint baseJoints,  std::vector<visualization_msgs::InteractiveMarker>& iMarkers, bool show_unreachable_models); //visualizing robot model with joint solutions
  bool makeRobotMarker(BasePoseJoint baseJoints,  std::vector<visualization_msgs::InteractiveMarker>& iMarkers, bool show_unreachable_models); //visualizing robot model with joint solutions
  bool checkForEE();



private:

  std::string group_name_;
  ros::AsyncSpinner spinner;
  boost::scoped_ptr<moveit::planning_interface::MoveGroup> group_;
  //Robot model cons pointer
  moveit::core::RobotModelConstPtr robot_model_;
};

#endif // CREATE_MARKER_H
