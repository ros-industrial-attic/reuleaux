#include<base_placement_plugin/add_robot_base.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>



AddRobotBase::AddRobotBase(QWidget *parent , std::string group_name)
{
  group_name_ = group_name;
  init();
}

AddRobotBase::~AddRobotBase()
{
  server.reset();
}

void AddRobotBase::init()
{

  server.reset(new interactive_markers::InteractiveMarkerServer("user_intutition", "", false));
  ros::Duration(0.1).sleep();
  server->applyChanges();
  waypoints_pos.clear();
  count = 0;

  ROBOT_WAY_POINT_COLOR.r = 0.5;
  ROBOT_WAY_POINT_COLOR.g = 0.5;
  ROBOT_WAY_POINT_COLOR.b = 0.5;
  ROBOT_WAY_POINT_COLOR.a = 0.3;

  WAY_POINT_COLOR.r = 0.1;
  WAY_POINT_COLOR.g = 0.5;
  WAY_POINT_COLOR.b = 1;
  WAY_POINT_COLOR.a = 1.0;

  WAY_POINT_SCALE_CONTROL.x = 0.28;
  WAY_POINT_SCALE_CONTROL.y = 0.032;
  WAY_POINT_SCALE_CONTROL.z = 0.032;

  INTERACTIVE_MARKER_SCALE = 0.4;

  ARROW_INTER_COLOR.r = 0.2;
  ARROW_INTER_COLOR.g = 0.2;
  ARROW_INTER_COLOR.b = 0.2;
  ARROW_INTER_COLOR.a = 1.0;

  ROBOT_INTER_COLOR.r = 0.0;
  ROBOT_INTER_COLOR.g = 1.0;
  ROBOT_INTER_COLOR.b = 0.0;
  ROBOT_INTER_COLOR.a = 0.5;


  ARROW_INTER_SCALE_CONTROL.x = 0.27;
  ARROW_INTER_SCALE_CONTROL.y = 0.03;
  ARROW_INTER_SCALE_CONTROL.z = 0.03;




  ARROW_INTERACTIVE_SCALE = 0.3;
  menu_handler.insert("Delete", boost::bind(&AddRobotBase::processFeedback, this, _1));
  menu_handler.setCheckState(menu_handler.insert("Fine adjustment", boost::bind(&AddRobotBase::processFeedback, this, _1)),
        interactive_markers::MenuHandler::UNCHECKED);


  mark_ = new CreateMarker(group_name_);
  robot_markers_ = mark_->getDefaultMarkers();



  //tf::Vector3 vec(-1, 0, 0);
  tf::Vector3 vec(-1, 0, 0);
  tf::Quaternion quat(0, 0, 0, 1);
  quat.normalize();
  tf::Transform trns;
  trns.setOrigin(vec);
  trns.setRotation(quat);

  box_pos = trns;
  target_frame_.assign("base_link");
  ROS_INFO_STREAM("The robot model frame is: " << target_frame_);
  makeInteractiveMarker();
  server->applyChanges();
  ROS_INFO("User base placement interactive marker started.");
}

void AddRobotBase::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  switch (feedback->event_type)
    {
      case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      {
        tf::Transform point_pos;
        tf::poseMsgToTF(feedback->pose, point_pos);
        // ROS_INFO_STREAM("on click feedback pose is"<<feedback->pose.position.x<<", "<<feedback->pose.position.y<<",
        // "<<feedback->pose.position.z<<";");

        makeArrow(point_pos, count);
        break;
      }

      case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      {
        tf::Transform point_pos;
        tf::poseMsgToTF(feedback->pose, point_pos);
        pointPoseUpdated(point_pos, feedback->marker_name.c_str());

        //Q_EMIT pointPoseUpdatedRViz(point_pos, feedback->marker_name.c_str());

        break;
      }
      case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      {
        // get the menu item which is pressed
        interactive_markers::MenuHandler::EntryHandle menu_item = feedback->menu_entry_id;
        interactive_markers::MenuHandler::CheckState state;

        menu_handler.getCheckState(menu_item, state);

        if (menu_item == 1)
        {
          std::string marker_name = feedback->marker_name;
          int marker_nr = atoi(marker_name.c_str());
          //Q_EMIT pointDeleteRviz(marker_nr);
          pointDeleted(marker_name);
          break;
        }
        else
        {
          if (state == interactive_markers::MenuHandler::UNCHECKED)
          {
            ROS_INFO("The selected marker is shown with 6DOF control");
            menu_handler.setCheckState(menu_item, interactive_markers::MenuHandler::CHECKED);
            geometry_msgs::Pose pose;
            changeMarkerControlAndPose(feedback->marker_name.c_str(), true);
            break;
          }
          else
          {
            menu_handler.setCheckState(menu_item, interactive_markers::MenuHandler::UNCHECKED);
            ROS_INFO("The selected marker is shown as default");
            geometry_msgs::Pose pose;
            changeMarkerControlAndPose(feedback->marker_name.c_str(), false);
            break;
          }
        }
        break;
      }
    }
    server->applyChanges();
}


void AddRobotBase::getWaypoints(std::vector<geometry_msgs::Pose> &waypoints)
{
  for(int i=0;i<waypoints_pos.size();i++)
  {
    geometry_msgs::Pose base_pose;
    tf::poseTFToMsg(waypoints_pos[i], base_pose);
    waypoints[i] =base_pose;
  }
}

void AddRobotBase::changeMarkerControlAndPose(std::string marker_name, bool set_control)
{
  visualization_msgs::InteractiveMarker int_marker;
  server->get(marker_name, int_marker);

  if (set_control)
  {
    int_marker.controls.clear();
    makeArrowControlDetails(int_marker);
  }
  else if (!set_control)
  {
    int_marker.controls.clear();
    makeArrowControlDefault(int_marker);
  }

  server->insert(int_marker);
  menu_handler.apply(*server, int_marker.name);
  server->setCallback(int_marker.name, boost::bind(&AddRobotBase::processFeedback, this, _1));
}

visualization_msgs::InteractiveMarkerControl& AddRobotBase::makeArrowControlDefault(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::InteractiveMarkerControl control_menu;
  control_menu.always_visible = true;

  control_menu.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;

  control_menu.name = "menu_select";
  msg.controls.push_back(control_menu);
  //control_menu.markers.push_back(makeWayPoint(msg));

  visualization_msgs::InteractiveMarkerControl control_move3d;
  control_move3d.always_visible = true;

  control_move3d.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  control_move3d.name = "move";
  control_move3d.markers.push_back(makeWayPoint(msg));

  visualization_msgs::MarkerArray control_move3d_markers = makeRobotMarker(msg, true);

  for(int i=0;i<control_move3d_markers.markers.size();++i)
    control_move3d.markers.push_back(control_move3d_markers.markers[i]);

  msg.controls.push_back(control_move3d);
  return msg.controls.back();
}



visualization_msgs::InteractiveMarkerControl& AddRobotBase::makeArrowControlDetails(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::InteractiveMarkerControl control_menu;
    control_menu.always_visible = true;

    control_menu.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    control_menu.name = "menu_select";
    msg.controls.push_back(control_menu);
    control_menu.markers.push_back(makeWayPoint(msg));

    visualization_msgs::MarkerArray control_menu_markers = makeRobotMarker(msg, true);
    for(int i=0;i<control_menu_markers.markers.size();++i)
      control_menu.markers.push_back(control_menu_markers.markers[i]);



    visualization_msgs::InteractiveMarkerControl control_view_details;
    control_view_details.always_visible = true;
    //*************rotate and move around the x-axis********************
    control_view_details.orientation.w = 1;
    control_view_details.orientation.x = 1;
    control_view_details.orientation.y = 0;
    control_view_details.orientation.z = 0;

    control_view_details.name = "move_x";
    control_view_details.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    msg.controls.push_back(control_view_details);
    //*****************************************************************

    //*************rotate and move around the z-axis********************
    control_view_details.orientation.w = 1;
    control_view_details.orientation.x = 0;
    control_view_details.orientation.y = 1;
    control_view_details.orientation.z = 0;

    control_view_details.name = "rotate_z";
    control_view_details.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    msg.controls.push_back(control_view_details);

    //*****************************************************************

    //*************rotate and move around the y-axis********************
    control_view_details.orientation.w = 1;
    control_view_details.orientation.x = 0;
    control_view_details.orientation.y = 0;
    control_view_details.orientation.z = 1;

    control_view_details.name = "move_y";
    control_view_details.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    msg.controls.push_back(control_view_details);
    control_view_details.markers.push_back(makeWayPoint(msg));

    visualization_msgs::MarkerArray control_view_markers = makeRobotMarker(msg, true);
    for(int i=0;i<control_view_markers.markers.size();++i)
      control_view_details.markers.push_back(control_view_markers.markers[i]);
    msg.controls.push_back(control_view_details);

    //*****************************************************************
    return msg.controls.back();
}

visualization_msgs::Marker AddRobotBase::makeWayPoint(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale = WAY_POINT_SCALE_CONTROL;

  marker.color = WAY_POINT_COLOR;
  marker.action = visualization_msgs::Marker::ADD;
  return marker;
}

void AddRobotBase::pointDeleted(std::string marker_name)
{
  for (int i = 0; i < waypoints_pos.size(); i++)
      ROS_DEBUG_STREAM("vecotr before delete: \n"
                       << "x:" << waypoints_pos[i].getOrigin().x() << "; " << waypoints_pos[i].getOrigin().y() << "; "
                       << waypoints_pos[i].getOrigin().z() << ";\n");

    // get the index of the selected marker
    int index = atoi(marker_name.c_str());
    server->erase(marker_name.c_str());
    waypoints_pos.erase(waypoints_pos.begin() + index - 1);

    for (int i = 0; i < waypoints_pos.size(); i++)
      ROS_DEBUG_STREAM("vecotr before delete: \n"
                       << "x:" << waypoints_pos[i].getOrigin().x() << "; " << waypoints_pos[i].getOrigin().y() << "; "
                       << waypoints_pos[i].getOrigin().z() << ";\n");
    // InteractiveMarker int_marker;
    for (int i = index + 1; i <= count; i++)
    {
      std::stringstream s;
      s << i;
      server->erase(s.str());
      makeArrow(waypoints_pos[i - 2], (i - 1));
    }
    count--;
    server->applyChanges();
}


void AddRobotBase::makeArrow(const tf::Transform& point_pos, int count_arrow)  //
{
  /*! Function for adding a new Way-Point in the RViz scene and here we send the signal to notify the RQT Widget that a
   * new Way-Point has been added.
  */
  visualization_msgs::InteractiveMarker int_marker;
  ROS_INFO_STREAM("Markers frame is: " << target_frame_);
  int_marker.header.frame_id = target_frame_;
  ROS_DEBUG_STREAM("Markers has frame id: " << int_marker.header.frame_id);
  int_marker.scale = INTERACTIVE_MARKER_SCALE;
  tf::poseTFToMsg(point_pos, int_marker.pose);
  std::vector< tf::Transform >::iterator it_pos =
      std::find((waypoints_pos.begin()), (waypoints_pos.end() - 1), point_pos);

  /*! Check the positions and orientations vector if they are emtpy. If it is empty we have our first Way-Point.
  */
  if (waypoints_pos.empty())
  {
    ROS_INFO("Adding first arrow!");
    count_arrow++;
    count = count_arrow;

    waypoints_pos.push_back(point_pos);
    //Q_EMIT addPointRViz(point_pos, count);
  }
  /*! Check if we have points in the same position in the scene. If we do, do not add one and notify the RQT Widget so
   * it can also add it to the TreeView.
  */
  else if ((it_pos == (waypoints_pos.end())) ||
           (point_pos.getOrigin() !=
            waypoints_pos.at(count_arrow - 1).getOrigin()))  // && (point_pos.getOrigin() !=
                                                             // waypoints_pos.at(count_arrow-1).getOrigin()) //(it_pos
                                                             // == waypoints_pos.end()) &&
  {
    count_arrow++;
    count = count_arrow;
    waypoints_pos.push_back(point_pos);
    ROS_INFO("Adding new arrow!");
    //Q_EMIT addPointRViz(point_pos, count);
  }
  else
  {
    // if we have arrow, ignore adding new one and inform the user that there is arrow (waypoint at that location)
    ROS_INFO("There is already a arrow at that location, can't add new one!!");
  }
  /*******************************************************************************************************************************************************************************************************************/
  std::stringstream s;
  s << count_arrow;
  ROS_DEBUG("end of make arrow, count is:%d, positions count:%ld", count, waypoints_pos.size());
  int_marker.name = s.str();
  int_marker.description = s.str();

  makeArrowControlDefault(int_marker);
  server->insert(int_marker);
  server->setCallback(int_marker.name, boost::bind(&AddRobotBase::processFeedback, this, _1));
  menu_handler.apply(*server, int_marker.name);
  // server->applyChanges();
  // Q_EMIT onUpdatePosCheckIkValidity(int_marker.pose,count_arrow);
}

void AddRobotBase::makeInteractiveMarker()
{
  visualization_msgs::InteractiveMarker inter_arrow_marker_;
  inter_arrow_marker_.header.frame_id = target_frame_;
  inter_arrow_marker_.scale = ARROW_INTERACTIVE_SCALE;
  ROS_INFO_STREAM("Marker Frame is:" << target_frame_);
  tf::poseTFToMsg(box_pos, inter_arrow_marker_.pose);
  inter_arrow_marker_.description = "Base Marker";
  // button like interactive marker. Detect when we have left click with the mouse and add new arrow then
  inter_arrow_marker_.name = "add_point_button";
  makeInteractiveMarkerControl(inter_arrow_marker_);
  server->insert(inter_arrow_marker_);
    // add interaction feedback to the markers
  server->setCallback(inter_arrow_marker_.name, boost::bind(&AddRobotBase::processFeedback, this, _1));
}

visualization_msgs::InteractiveMarkerControl& AddRobotBase::makeInteractiveMarkerControl(visualization_msgs::InteractiveMarker& msg)
{
  visualization_msgs::InteractiveMarkerControl control_button;
  control_button.always_visible = true;
  control_button.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control_button.name = "button_interaction";
  control_button.markers.push_back(makeInterArrow(msg));
  visualization_msgs::MarkerArray control_button_markers = makeRobotMarker(msg, false);
  for(int i=0;i<control_button_markers.markers.size();++i)
    control_button.markers.push_back(control_button_markers.markers[i]);

  msg.controls.push_back(control_button);
  visualization_msgs::InteractiveMarkerControl control_inter_arrow;
  control_inter_arrow.always_visible = true;

    //*************rotate and move around the x-axis********************
  control_inter_arrow.orientation.w = 1;
  control_inter_arrow.orientation.x = 1;
  control_inter_arrow.orientation.y = 0;
  control_inter_arrow.orientation.z = 0;
  control_inter_arrow.name = "move_x";
  control_inter_arrow.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_inter_arrow);
  //*****************************************************************

  //*************rotate and move around the y-axis********************
  control_inter_arrow.orientation.w = 1;
  control_inter_arrow.orientation.x = 0;
  control_inter_arrow.orientation.y = 0;
  control_inter_arrow.orientation.z = 1;
  control_inter_arrow.name = "move_y";
  control_inter_arrow.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_inter_arrow);
  //*****************************************************************


  //*************rotate and move around the z-axis********************
  control_inter_arrow.orientation.w = 1;
  control_inter_arrow.orientation.x = 0;
  control_inter_arrow.orientation.y = 1;
  control_inter_arrow.orientation.z = 0;
  control_inter_arrow.name = "rotate_z";
  control_inter_arrow.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_inter_arrow);
  //*****************************************************************

  control_inter_arrow.markers.push_back(makeInterArrow(msg));
  return msg.controls.back();
}

visualization_msgs::MarkerArray AddRobotBase::makeRobotMarker(visualization_msgs::InteractiveMarker& msg, bool waypoint)
{
  Eigen::Affine3d base_tf;
  tf::poseMsgToEigen(msg.pose, base_tf);
  visualization_msgs::MarkerArray markArr;
  visualization_msgs::MarkerArray robot_markers_new = robot_markers_;
  for(int i=0;i<robot_markers_.markers.size();++i)
  {
    robot_markers_new.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
    robot_markers_new.markers[i].mesh_use_embedded_materials = true;
    if(waypoint)
      robot_markers_new.markers[i].color = ROBOT_WAY_POINT_COLOR;
    else
      robot_markers_new.markers[i].color = ROBOT_INTER_COLOR;

    robot_markers_new.markers[i].action =  visualization_msgs::Marker::ADD;

    Eigen::Affine3d link_marker;
    tf::poseMsgToEigen(robot_markers_.markers[i].pose, link_marker);
    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg(base_tf * link_marker, new_marker_pose);
    robot_markers_new.markers[i].pose = new_marker_pose;

    markArr.markers.push_back(robot_markers_new.markers[i]);
  }
  return markArr;
}


visualization_msgs::Marker AddRobotBase::makeInterArrow(visualization_msgs::InteractiveMarker& msg)
{
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale = ARROW_INTER_SCALE_CONTROL;
  // make the markers with interesting color
  marker.color = ARROW_INTER_COLOR;
  return marker;
}

void AddRobotBase::parseWayPoints()
{
  geometry_msgs::Pose target_pose;
  std::vector<geometry_msgs::Pose> waypoints;
  for(int i=0;i<waypoints_pos.size();i++)
  {
    tf::poseTFToMsg(waypoints_pos[i], target_pose);
    waypoints.push_back(target_pose);
  }
  Q_EMIT baseWayPoints_signal(waypoints);
}


void AddRobotBase::clearAllPointsRviz()
{
  waypoints_pos.clear();
  server->clear();
  // delete the waypoints_pos vector
  count = 0;
  makeInteractiveMarker();
  server->applyChanges();
}

void AddRobotBase::getRobotModelFrame_slot(const tf::Transform end_effector)
{
  target_frame_.assign("base_link");
  ROS_INFO_STREAM("The robot model frame is: " << target_frame_);
  box_pos = end_effector;
  clearAllPointsRviz();
  count = 0;
  makeInteractiveMarker();
  server->applyChanges();
}

void AddRobotBase::pointPoseUpdated(const tf::Transform &point_pos, const char *marker_name)
{
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(point_pos, pose);
  std::stringstream s;
  if (strcmp("add_point_button", marker_name) == 0)
  {
    box_pos = point_pos;
    s << "add_point_button";
  }
  else
  {
    int index = atoi(marker_name);
    if(index>waypoints_pos.size())
    {
      return;
    }
    waypoints_pos[index-1] = point_pos;
    s<<index;
  }
  server->setPose(s.str(), pose);
  server->applyChanges();

}


