#include<base_placement_plugin/create_marker.h>
#include <ctime>
#include<typeinfo>

double unifRand()
{
    return rand() / double(RAND_MAX);
}

CreateMarker::CreateMarker(std::string group_name) : spinner(1), group_name_(group_name)
{
  spinner.start();
  group_.reset(new MoveGroupInterface(group_name_));
  //ROS_INFO_STREAM("Selected planning group: "<< group_->getName());
  robot_model_ = group_->getRobotModel();
}

bool CreateMarker::checkEndEffector()
{
  if(!(group_->getEndEffector()).empty())
    return true;
  else
    return false;
}

void CreateMarker::discardUnreachableModels(BasePoseJoint& baseJoints)
{
  for(BasePoseJoint::iterator it= baseJoints.begin(); it !=baseJoints.end();)
  {
    std::vector<double> joint_soln = it->first;
    if(std::equal(joint_soln.begin()+1, joint_soln.end(), joint_soln.begin()))
    {
      BasePoseJoint::iterator save = it;
      ++save;
      baseJoints.erase(it);
      it = save;
    }
    else
      ++it;
  }
}

bool checkForJointSoln(const std::vector<double>& soln)
{
  if(std::equal(soln.begin()+1, soln.end(), soln.begin()))
    return true;
}

void CreateMarker::updateRobotState(const std::vector<double>& joint_soln, moveit::core::RobotStatePtr robot_state)
{
  std::string robot_name = group_->getName();
  const moveit::core::JointModelGroup* robot_jmp = robot_model_->getJointModelGroup(robot_name);
  std::vector<std::string> joint_names = robot_jmp->getActiveJointModelNames();
  for (int i=0;i<joint_soln.size();i++)
  {
    robot_state->setJointPositions(joint_names[i], &(joint_soln[i]));
  }
  robot_state->update();
}

void CreateMarker::getArmLinks(std::vector<std::string>& arm_links)
{
  std::string arm_name = group_->getName();
  const moveit::core::JointModelGroup* arm_jmp = robot_model_->getJointModelGroup(arm_name);
  arm_links = arm_jmp->getLinkModelNames();
}

void CreateMarker::getEELinks(std::vector<std::string>& ee_links)
{
  std::string ee_name = group_->getEndEffector();
  if(robot_model_->hasJointModelGroup(ee_name))
  {
    const moveit::core::JointModelGroup* ee_jmp = robot_model_->getJointModelGroup(ee_name);
    ee_links = ee_jmp->getLinkModelNames();
  }
}

void CreateMarker::getFullLinkNames(std::vector<std::string>& full_link_names, bool arm_only)
{
  std::vector<std::string> full_links = robot_model_->getLinkModelNames();
  if(arm_only)
  {
    std::vector<std::string> arm_links, ee_links;
    getArmLinks(arm_links);
    getEELinks(ee_links);
    int position = std::find(full_links.begin(), full_links.end(), arm_links[0]) - full_links.begin();
    parent_link = full_links[position -1];
    for(int i=0;i<arm_links.size();++i)
      full_link_names.push_back(arm_links[i]);
    for(int i=0;i<ee_links.size();++i)
      full_link_names.push_back(ee_links[i]);
  }
  else
  {
    full_link_names = full_links;
    parent_link = full_link_names[0];
  }
}

void CreateMarker::updateMarkers(const geometry_msgs::Pose& base_pose, bool is_reachable, Eigen::Affine3d tf_first_link_to_root, visualization_msgs::MarkerArray& markers)
{
  Eigen::Affine3d base_tf;
  tf::poseMsgToEigen(base_pose, base_tf);
  double r = unifRand();
  double g = unifRand();
  double b = unifRand();
  for(std::size_t j=0;j<markers.markers.size();j++)
  {
    markers.markers[j].header.frame_id = "base_link";
    markers.markers[j].type = visualization_msgs::Marker::MESH_RESOURCE;
    markers.markers[j].mesh_use_embedded_materials = true;
    markers.markers[j].id = j*5;
    markers.markers[j].header.stamp = ros::Time::now();
    markers.markers[j].ns = "robot_links";
    markers.markers[j].lifetime = ros::Duration(40.0);
    if(!is_reachable)
    {
      markers.markers[j].color.r = 1.0;
      markers.markers[j].color.g = 0.0;
      markers.markers[j].color.b = 0.0;
      markers.markers[j].color.a= 1.0;
    }
    else
    {
      markers.markers[j].color.r = r;
      markers.markers[j].color.g = g;
      markers.markers[j].color.b = b;
      markers.markers[j].color.a= 0.7;
    }
    Eigen::Affine3d link_marker;
    tf::poseMsgToEigen(markers.markers[j].pose, link_marker);
    Eigen::Affine3d tf_link_in_root = tf_first_link_to_root * link_marker;
    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg(base_tf * tf_link_in_root, new_marker_pose);
    markers.markers[j].pose = new_marker_pose;
  }
}


void CreateMarker::makeIntMarkerControl(const geometry_msgs::Pose& base_pose, const std::vector<double>& joint_soln,bool arm_only, bool is_reachable, visualization_msgs::InteractiveMarkerControl& robotModelControl)
{
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model_));
  updateRobotState(joint_soln, robot_state);
  std::vector<std::string> full_link_names;
  getFullLinkNames(full_link_names, arm_only);
  visualization_msgs::MarkerArray full_link_markers;
  robot_state->getRobotMarkers(full_link_markers, full_link_names);

  //const std::string& parent_link = full_link_names[0]; //root link
  Eigen::Affine3d tf_root_to_first_link = robot_state->getGlobalLinkTransform(parent_link);
  Eigen::Affine3d tf_first_link_to_root = tf_root_to_first_link.inverse();

  updateMarkers(base_pose, is_reachable, tf_first_link_to_root, full_link_markers);
  for(int i=0;i<full_link_markers.markers.size();++i)
    robotModelControl.markers.push_back(full_link_markers.markers[i]);
  robotModelControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  robotModelControl.always_visible = true;
}


void CreateMarker::createInteractiveMarker(const geometry_msgs::Pose& base_pose, const std::vector<double>& joint_soln,
                                           const int& num, bool arm_only, bool is_reachable,visualization_msgs::InteractiveMarker& iMarker)
{
  iMarker.header.frame_id = "base_link";
  iMarker.pose = base_pose;
  iMarker.scale = 0.2;
  std::string name = "robot_model";
  std::string description = "robot_desc";
  iMarker.name = name + boost::lexical_cast<std::string>(num);
  iMarker.description = description+boost::lexical_cast<std::string>(num);
  visualization_msgs::InteractiveMarkerControl robotModelControl;
  makeIntMarkerControl(base_pose, joint_soln, arm_only, is_reachable,robotModelControl);
  iMarker.controls.push_back(robotModelControl);
}

void CreateMarker::makeIntMarkers(BasePoseJoint &basePJoints, bool arm_only, std::vector<visualization_msgs::InteractiveMarker> &iMarkers)
{
  iMarkers.clear();
  for(BasePoseJoint::iterator it = basePJoints.begin(); it !=basePJoints.end();++it)
  {
    int i = std::distance(basePJoints.begin(), it);
    bool is_reachable = true;
    geometry_msgs::Pose base_pose = it->second;
    std::vector<double> joint_soln = it->first;
    if(checkForJointSoln(joint_soln))
      is_reachable = false;
    visualization_msgs::InteractiveMarker iMarker;
    createInteractiveMarker(base_pose, joint_soln, i, arm_only, is_reachable, iMarker );
    iMarkers.push_back(iMarker);
  }
}


bool CreateMarker::makeRobotMarker(BasePoseJoint baseJoints, std::vector<visualization_msgs::InteractiveMarker> &iMarkers, bool show_unreachable_models)
{
 if(!show_unreachable_models)
     discardUnreachableModels(baseJoints);
  makeIntMarkers(baseJoints, false, iMarkers);
}


bool CreateMarker::makeArmMarker(BasePoseJoint baseJoints, std::vector<visualization_msgs::InteractiveMarker> &iMarkers, bool show_unreachable_models)
{
  if(!show_unreachable_models)
      discardUnreachableModels(baseJoints);
   makeIntMarkers(baseJoints, true, iMarkers);
}


visualization_msgs::MarkerArray CreateMarker::getDefaultMarkers()
{
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model_));
  //updateRobotState(joint_soln, robot_state);
  std::vector<std::string> full_link_names;
  getFullLinkNames(full_link_names, false);
  visualization_msgs::MarkerArray full_link_markers;
  robot_state->getRobotMarkers(full_link_markers, full_link_names);
  return full_link_markers;
}



