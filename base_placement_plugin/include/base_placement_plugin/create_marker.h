#ifndef CREATE_MARKER_H
#define CREATE_MARKER_H

#include<ros/ros.h>
#include<geometry_msgs/Pose.h>
#include<visualization_msgs/InteractiveMarker.h>
#include<tf2/LinearMath/Transform.h>
#include<tf2/LinearMath/Quaternion.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

#include<moveit/move_group_interface/move_group.h>
#include<moveit/robot_state/robot_state.h>

#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit/robot_model/robot_model.h>
#include<moveit/robot_model/joint_model_group.h>
#include<moveit/robot_model/link_model.h>


typedef std::multimap<std::vector<double>,geometry_msgs::Pose> BasePoseJoint;

class CreateMarker{
public:
  CreateMarker(std::string group_name);


  bool makeArmMarker(BasePoseJoint baseJoints,  std::vector<visualization_msgs::InteractiveMarker>& iMarkers, bool show_unreachable_models); //visualizing robot model with joint solutions
  bool makeRobotMarker(BasePoseJoint baseJoints,  std::vector<visualization_msgs::InteractiveMarker>& iMarkers, bool show_unreachable_models); //visualizing robot model with joint solutions
  bool checkEndEffector();

  visualization_msgs::MarkerArray getDefaultMarkers();




private:

  void discardUnreachableModels(BasePoseJoint& baseJoints);
  void makeIntMarkers(BasePoseJoint& basePJoints, bool arm_only, std::vector<visualization_msgs::InteractiveMarker> &iMarkers);
  void updateRobotState(const std::vector<double>& joint_soln, moveit::core::RobotStatePtr robot_state);
  void getFullLinkNames(std::vector<std::string>& full_link_names, bool arm_only);
  void getArmLinks(std::vector<std::string>& arm_links);
  void getEELinks(std::vector<std::string>& ee_links);
  void makeIntMarkerControl(const geometry_msgs::Pose& base_pose, const std::vector<double>& joint_soln,bool arm_only, bool is_reachable, visualization_msgs::InteractiveMarkerControl& robotModelControl);
  void createInteractiveMarker(const geometry_msgs::Pose& base_pose, const std::vector<double>& joint_soln,
                                             const int& num, bool arm_only, bool is_reachable,visualization_msgs::InteractiveMarker& iMarker);

  void updateMarkers(const geometry_msgs::Pose& base_pose, bool is_reachable, Eigen::Affine3d tf_first_link_to_root, visualization_msgs::MarkerArray& markers);



  std::string group_name_;
  ros::AsyncSpinner spinner;
  boost::scoped_ptr<moveit::planning_interface::MoveGroup> group_;
  std::string parent_link;
  moveit::core::RobotModelConstPtr robot_model_; //Robot model const pointer
};

#endif // CREATE_MARKER_H
