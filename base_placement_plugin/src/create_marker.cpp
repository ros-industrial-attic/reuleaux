#include<base_placement_plugin/create_marker.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <ctime>
#include<moveit/robot_model/link_model.h>

double unifRand()
{
    return rand() / double(RAND_MAX);
}

CreateMarker::CreateMarker(std::string group_name) : spinner(1), group_name_(group_name)
{
  spinner.start();
  group_.reset(new moveit::planning_interface::MoveGroup(group_name_));
  ROS_INFO_STREAM("Selected planning group: "<< group_->getName());
  robot_model_ = group_->getRobotModel();

}

bool CreateMarker::checkForEE()
{
  std::vector<std::string> groups = robot_model_->getJointModelGroupNames();
  for(int i=0;i<groups.size();i++)
  {
    if(robot_model_->hasEndEffector(groups[i]))
    {
      return true;
    }
    else
      return false;
  }
}

bool CreateMarker::makeArmMarker( BasePoseJoint baseJoints, std::vector<visualization_msgs::InteractiveMarker> &iMarkers, bool show_unreachable_models)
{
  iMarkers.clear();//Clearing previous markers
  std::string robot_name = group_->getName();//obtaining group name
  bool has_joint_model_ee = false;
  std::vector<std::string> ee_links;

  if(!robot_model_->hasJointModelGroup(robot_name) )
  {
    ROS_INFO("OOPS.either be you have selected your endeffector as robot model or your arm does not have any joint model group. Please wait until the task is finished and change it");
    return false;
  }


  if(!show_unreachable_models) //Then delete all unreachable models so it wont consume memory
  {
    for (BasePoseJoint::iterator it = baseJoints.begin(); it !=baseJoints.end();)
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

  //checking for end-effector
  if(checkForEE())
  {
    has_joint_model_ee = true;
    std::string ee_name = group_->getEndEffector();
    if(ee_name.size() ==0)
    {
      ROS_INFO("OOps. you have selected the end effector as robotmodel to show. Please wait until the task is finished and change it");
      return false;
    }
    const moveit::core::JointModelGroup* ee_jmp = robot_model_->getJointModelGroup(ee_name);
    const std::vector<std::string>& ee_link_names = ee_jmp->getLinkModelNames();
    for(int i=0;i<ee_link_names.size();i++)
    {
      ee_links.push_back(ee_link_names[i]);
    }
  }

  const moveit::core::JointModelGroup* robot_jmp = robot_model_->getJointModelGroup(robot_name);//Obtaining the jointmodel group of the arm
  std::vector<std::string> joint_names = robot_jmp->getActiveJointModelNames();//obtaining all the moveable joint names in the arm
  for (BasePoseJoint::iterator it = baseJoints.begin(); it !=baseJoints.end(); ++it)//Creating each interactive marker
  {
    int i=std::distance(baseJoints.begin(), it);
    bool no_joint_solution = false;
    geometry_msgs::Pose base_pose = it->second;
    std::vector<double> joint_soln = it->first;
    if(std::equal(joint_soln.begin()+1, joint_soln.end(), joint_soln.begin()))
    {
          no_joint_solution = true;
    }
    moveit::core::RobotStatePtr robot_state_(new moveit::core::RobotState(robot_model_));//Calling robot_state in each iteration
    for(int k=0;k<joint_soln.size();k++)
    {
      robot_state_->setJointPositions(joint_names[k], &(joint_soln[k])); //Setting joint values to arm joints
    }
    robot_state_->update(); //Updating robot state
    const std::vector<std::string>& arm_link_names = robot_jmp->getLinkModelNames();//Obtaining all the arm link names
    std::vector<std::string> full_link_names = robot_model_->getLinkModelNames(); //Obtaing all the robot model names
    int position = std::find(full_link_names.begin(), full_link_names.end(), arm_link_names[0]) -full_link_names.begin() ;
    std::string arm_parent_link = full_link_names[position-1];
    std::vector<std::string> full_links;
    for(int k=0;k<arm_link_names.size();k++)
    {
      full_links.push_back(arm_link_names[k]);
    }

    if(has_joint_model_ee)
    {
      for(int k=0;k<ee_links.size();k++)
      {
        full_links.push_back(ee_links[k]);
      }
    }

    visualization_msgs::MarkerArray full_link_markers;
    robot_state_->getRobotMarkers( full_link_markers, full_links);//Obtain the current markers

    Eigen::Affine3d tf_root_to_first_link = robot_state_->getGlobalLinkTransform(arm_parent_link);//transform of the parent link
    Eigen::Affine3d tf_first_link_to_root = tf_root_to_first_link.inverse();//inverse of the transform

    Eigen::Affine3d base_tf;
    tf::poseMsgToEigen(base_pose, base_tf);//base pose in tf
    visualization_msgs::InteractiveMarker iMarker; //new interactive marker
    iMarker.header.frame_id = "base_link";
    iMarker.pose = base_pose;//the pose of the imarker is the pose of the base
    iMarker.scale = 0.2;
    std::string name = "robot_model";
    std::string description = "robot model";
    iMarker.name = name+boost::lexical_cast<std::string>(i);
    iMarker.description = description+boost::lexical_cast<std::string>(i);//each imarker has different name and description
    visualization_msgs::InteractiveMarkerControl robotModelControl;
    for(std::size_t j=0; j<full_link_markers.markers.size();j++)
    {
      full_link_markers.markers[j].header.frame_id = "base_link";
      full_link_markers.markers[j].type = visualization_msgs::Marker::MESH_RESOURCE;
      full_link_markers.markers[j].mesh_use_embedded_materials = true;
      full_link_markers.markers[j].id = i*(j+5);
      full_link_markers.markers[j].header.stamp = ros::Time::now();
      full_link_markers.markers[j].ns = "robot_links";
      full_link_markers.markers[j].lifetime = ros::Duration(40.0);
      //Just setting random color.Can be ignored
      if(no_joint_solution)
      {
        full_link_markers.markers[j].color.r = 1.0;
        full_link_markers.markers[j].color.g = 0.0;
        full_link_markers.markers[j].color.b = 0.0;
        full_link_markers.markers[j].color.a = 0.2;
      }
      else
      {
        if(j==1 || j==2 || j==5 || j==6 || j==9)
        {
          full_link_markers.markers[j].color.r = 0.1;
          full_link_markers.markers[j].color.g = 0.1;
          full_link_markers.markers[j].color.b = 0.1;
          full_link_markers.markers[j].color.a = 1.0;
        }
        else
        {
          full_link_markers.markers[j].color.r = 0.5;
          full_link_markers.markers[j].color.g = 0.5;
          full_link_markers.markers[j].color.b = 0.5;
          full_link_markers.markers[j].color.a = 1.0;
        }
      }
      Eigen::Affine3d link_marker;
      tf::poseMsgToEigen(full_link_markers.markers[j].pose, link_marker);//pose of each marker
      Eigen::Affine3d tf_link_in_root =  tf_first_link_to_root * link_marker;//transformed by inverse transformation
      geometry_msgs::Pose new_marker_pose;
      tf::poseEigenToMsg( base_tf * tf_link_in_root  , new_marker_pose );//trasnformed to basepose
      visualization_msgs::Marker Marker =  full_link_markers.markers[j];
      Marker.pose = new_marker_pose;
      robotModelControl.markers.push_back(Marker);
      robotModelControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
      robotModelControl.always_visible = true;
      iMarker.controls.push_back(robotModelControl);
    }
  iMarkers.push_back(iMarker);
  }
}


bool CreateMarker::makeRobotMarker( BasePoseJoint baseJoints, std::vector<visualization_msgs::InteractiveMarker> &iMarkers, bool show_unreachable_models)
{
  iMarkers.clear();//Clearing previous markers
  std::string robot_name = group_->getName();//obtaining group name
  if(!robot_model_->hasJointModelGroup(robot_name))
  {
    ROS_INFO("OOPS.either be you have selected your endeffector as robot model or your arm does not have any joint model group. Please wait until task is finished and change it");
    return false;
  }
  //checking for end-effector
  if(checkForEE())
  {
    std::string ee_name = group_->getEndEffector();
    if(ee_name.size() ==0)
    {
      ROS_INFO("OOps. you have selected the end effector as robotmodel to show. Please wait until task is finished and change it");
      return false;
    }
  }

  if(!show_unreachable_models) //Then delete all unreachable models so it wont consume memory
  {
    for (BasePoseJoint::iterator it = baseJoints.begin(); it !=baseJoints.end();)
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

  //moveit::core::RobotModelConstPtr robot_model = group_->getRobotModel();//obtaining the robot model
  const moveit::core::JointModelGroup* robot_jmp = robot_model_->getJointModelGroup(robot_name);//Obtaining the jointmodel group of the arm
  std::vector<std::string> joint_names = robot_jmp->getActiveJointModelNames();//obtaining all the moveable joint names in the arm
  for (BasePoseJoint::iterator it = baseJoints.begin(); it !=baseJoints.end(); ++it)//Creating each interactive marker
  {
    int i=std::distance(baseJoints.begin(), it);
    bool no_joint_solution = false; //flag for checking if for this robot model no ik solution exist
    geometry_msgs::Pose base_pose = it->second;
    std::vector<double> joint_soln = it->first;
    if(std::equal(joint_soln.begin()+1, joint_soln.end(), joint_soln.begin()))
    {
      no_joint_solution = true;
    }

    moveit::core::RobotStatePtr robot_state_(new moveit::core::RobotState(robot_model_));//Calling robot_state in each iteration
    for(int k=0;k<joint_soln.size();k++)
    {
      robot_state_->setJointPositions(joint_names[k], &(joint_soln[k])); //Setting joint values to arm joints
    }
    robot_state_->update();
    std::vector<std::string> full_link_names = robot_model_->getLinkModelNames(); //Collection of all the links for arm and gripper
    //ROS_INFO("Number of links: %lu ",full_link_names.size());
    visualization_msgs::MarkerArray full_link_markers;
    robot_state_->getRobotMarkers( full_link_markers, full_link_names);//Obtain the current markers
    //ROS_INFO("Number of markers: %lu",full_link_markers.markers.size());

    const std::string& ee_parent_link_name =  full_link_names[0];//root link of the arm
    Eigen::Affine3d tf_root_to_first_link = robot_state_->getGlobalLinkTransform(ee_parent_link_name);//transform of the root link
    Eigen::Affine3d tf_first_link_to_root = tf_root_to_first_link.inverse();//inverse of the transform
    Eigen::Affine3d base_tf;
    tf::poseMsgToEigen(base_pose, base_tf);//base pose in tf

    visualization_msgs::InteractiveMarker iMarker; //new interactive marker
    iMarker.header.frame_id = "base_link";
    iMarker.pose = base_pose;//the pose of the imarker is the pose of the base
    iMarker.scale = 0.2;
    std::string name = "robot_model";
    std::string description = "robot model";
    iMarker.name = name+boost::lexical_cast<std::string>(i);
    iMarker.description = description+boost::lexical_cast<std::string>(i);//each imarker has different name and description
    visualization_msgs::InteractiveMarkerControl robotModelControl;
    //Setting color for individual robot
    double r = unifRand();
    double g = unifRand();
    double b = unifRand();
    for(std::size_t j=0; j<full_link_markers.markers.size();j++)
    {
      full_link_markers.markers[j].header.frame_id = "base_link";
      full_link_markers.markers[j].type = visualization_msgs::Marker::MESH_RESOURCE;
      full_link_markers.markers[j].mesh_use_embedded_materials = true;
      full_link_markers.markers[j].id = i*(j+5);
      full_link_markers.markers[j].header.stamp = ros::Time::now();
      full_link_markers.markers[j].ns = "robot_links";
      full_link_markers.markers[j].lifetime = ros::Duration(40.0);
      if(no_joint_solution)
      {
        full_link_markers.markers[j].color.r = 1.0;
        full_link_markers.markers[j].color.g = 0.0;
        full_link_markers.markers[j].color.b = 0.0;
        full_link_markers.markers[j].color.a = 0.1;
      }
      else
      {
        full_link_markers.markers[j].color.r = r;
        full_link_markers.markers[j].color.g = g;
        full_link_markers.markers[j].color.b = b;
        full_link_markers.markers[j].color.a = 1;
      }

      Eigen::Affine3d link_marker;
      tf::poseMsgToEigen(full_link_markers.markers[j].pose, link_marker);//pose of each marker
      Eigen::Affine3d tf_link_in_root =  tf_first_link_to_root * link_marker;//transformed by inverse transformation
      geometry_msgs::Pose new_marker_pose;
      tf::poseEigenToMsg( base_tf * tf_link_in_root  , new_marker_pose );//trasnformed to basepose
      visualization_msgs::Marker Marker =  full_link_markers.markers[j];
      Marker.pose = new_marker_pose;
      robotModelControl.markers.push_back(Marker);
      robotModelControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
      robotModelControl.always_visible = true;
      iMarker.controls.push_back(robotModelControl);
    }
  iMarkers.push_back(iMarker);
  }
}





