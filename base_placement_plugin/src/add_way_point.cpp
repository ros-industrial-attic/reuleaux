#include <base_placement_plugin/add_way_point.h>

namespace base_placement_plugin
{
AddWayPoint::AddWayPoint(QWidget* parent) : rviz::Panel(parent)  //, tf_()
{
  /*!  The constructor sets the Object name, resets the Interactive Marker server.
       It initialize the subscriber to the mouse click topic and registers the call back to a mouse click event.
       It initializes the constants for the Marker color and scales, all the Interactive Markers are defined as arrows.
       The User Interaction arrow is set to have red color, The task poses are magenta

  */
  setObjectName("BasePlacementPlannerPlugin");
  server.reset(new interactive_markers::InteractiveMarkerServer("base_placement_plugin", "", false));

  WAY_POINT_COLOR.r = 1;
  WAY_POINT_COLOR.g = 0.20;
  WAY_POINT_COLOR.b = 1;
  WAY_POINT_COLOR.a = 1.0;

  WAY_POINT_SCALE_CONTROL.x = 0.28;
  WAY_POINT_SCALE_CONTROL.y = 0.032;
  WAY_POINT_SCALE_CONTROL.z = 0.032;

  INTERACTIVE_MARKER_SCALE = 0.4;

  ARROW_INTER_COLOR.r = 0.8;
  ARROW_INTER_COLOR.g = 0.2;
  ARROW_INTER_COLOR.b = 0.1;
  ARROW_INTER_COLOR.a = 1.0;

  ARROW_INTER_SCALE_CONTROL.x = 0.27;
  ARROW_INTER_SCALE_CONTROL.y = 0.03;
  ARROW_INTER_SCALE_CONTROL.z = 0.03;

  ARROW_INTERACTIVE_SCALE = 0.3;

  ROS_INFO("Constructor created;");
}

AddWayPoint::~AddWayPoint()
{
  /*! The object destructor resets the Interactive Marker server on the Object Destruction. */
  server.reset();
}

void AddWayPoint::onInitialize()
{
  /*!  Creating main layout object, object for the Base Placement Planning Class and the RQT Widget.
       Here we also create the Interactive Marker Menu handler for the Way-Points.
       Make all the necessary connections for the QObject communications.
       Inter Object connections for communications between the classes.
   */

  place_base = new PlaceBase();

  widget_ = new widgets::BasePlacementWidget("~");
  this->parentWidget()->resize(widget_->width(), widget_->height());
  QHBoxLayout* main_layout = new QHBoxLayout(this);
  main_layout->addWidget(widget_);

  //! Inform the user that the RViz is initializing
  ROS_INFO("initializing..");

  menu_handler.insert("Delete", boost::bind(&AddWayPoint::processFeedback, this, _1));
  menu_handler.setCheckState(
      menu_handler.insert("Fine adjustment", boost::bind(&AddWayPoint::processFeedback, this, _1)),
      interactive_markers::MenuHandler::UNCHECKED);

  // Place base to this

  connect(place_base, SIGNAL(getinitialmarkerFrame_signal(const tf::Transform)), this,
          SLOT(getRobotModelFrame_slot(const tf::Transform)));

  // Place base to widget

  connect(place_base, SIGNAL(getinitialmarkerFrame_signal(const tf::Transform)), widget_,
          SLOT(setAddPointUIStartPos(const tf::Transform)));
  connect(place_base, SIGNAL(basePlacementProcessStarted()), widget_, SLOT(PlaceBaseStartedHandler()));
  connect(place_base, SIGNAL(basePlacementProcessFinished()), widget_, SLOT(PlaceBaseFinishedHandler()));
  connect(place_base, SIGNAL(basePlacementProcessCompleted()), widget_, SLOT(PlaceBaseCompleted_slot()));
  connect(place_base, SIGNAL(sendBasePlaceMethods_signal(std::vector< std::string >)), widget_,
          SLOT(getBasePlacePlanMethod(std::vector< std::string >)));
  connect(place_base, SIGNAL(sendOuputType_signal(std::vector< std::string >)), widget_,
          SLOT(getOutputType(std::vector< std::string >)));

  // Widget to place base
  connect(widget_, SIGNAL(basePlacementParamsFromUI_signal(int, int)), place_base, SLOT(setBasePlaceParams(int, int)));
  connect(widget_, SIGNAL(reachabilityData_signal(std::multimap< std::vector< double >, std::vector< double > >,
                                                  std::multimap< std::vector< double >, double >, float)),
          place_base, SLOT(setReachabilityData(std::multimap< std::vector< double >, std::vector< double > >,
                                               std::multimap< std::vector< double >, double >, float)));
  connect(widget_, SIGNAL(showUnionMap_signal(bool)), place_base, SLOT(ShowUnionMap(bool)));
  connect(widget_, SIGNAL(clearUnionMap_signal(bool)), place_base, SLOT(clearUnionMap(bool)));
  connect(widget_, SIGNAL(SendSelectedMethod(int)), place_base, SLOT(getSelectedMethod(int)));
  connect(widget_, SIGNAL(SendSelectedOpType(int)), place_base, SLOT(getSelectedOpType(int)));

  // Widget to this

  connect(widget_, SIGNAL(addPoint(tf::Transform)), this, SLOT(addPointFromUI(tf::Transform)));
  connect(widget_, SIGNAL(pointDelUI_signal(std::string)), this, SLOT(pointDeleted(std::string)));
  connect(widget_, SIGNAL(parseWayPointBtn_signal()), this, SLOT(parseWayPoints()));
  connect(widget_, SIGNAL(pointPosUpdated_signal(const tf::Transform&, const char*)), this,
          SLOT(pointPoseUpdated(const tf::Transform&, const char*)));
  connect(widget_, SIGNAL(saveToFileBtn_press()), this, SLOT(saveWayPointsToFile()));
  connect(widget_, SIGNAL(clearAllPoints_signal()), this, SLOT(clearAllPointsRViz()));

  // This to widget
  connect(this, SIGNAL(addPointRViz(const tf::Transform&, const int)), widget_,
          SLOT(insertRow(const tf::Transform&, const int)));
  connect(this, SIGNAL(pointPoseUpdatedRViz(const tf::Transform&, const char*)), widget_,
          SLOT(pointPosUpdated_slot(const tf::Transform&, const char*)));
  connect(this, SIGNAL(pointDeleteRviz(int)), widget_, SLOT(removeRow(int)));

  // This to place base

  connect(this, SIGNAL(wayPoints_signal(std::vector< geometry_msgs::Pose >)), place_base,
          SLOT(BasePlacementHandler(std::vector< geometry_msgs::Pose >)));
  connect(this, SIGNAL(initRviz()), place_base, SLOT(initRvizDone()));

  /*!  With the signal initRviz() we call a function PlaceBase::initRvizDone() which sets the initial parameters of the
     MoveIt enviroment.

  */
  Q_EMIT initRviz();
  ROS_INFO("ready.");
}

void AddWayPoint::load(const rviz::Config& config)
{
  /*! \brief Setting up the configurations for the Panel of the RViz enviroment.
   */
  rviz::Panel::load(config);
  QString text_entry;
  ROS_INFO_STREAM("rviz: Initializing the user interaction planning panel");
  if (config.mapGetString("TextEntry", &text_entry))
  {
    ROS_INFO_STREAM("Loaded TextEntry with value: " << text_entry.toStdString());
  }

  ROS_INFO_STREAM("rviz Initialization Finished reading config file");
}

void AddWayPoint::save(rviz::Config config) const
{
  /// Allowing the user to save the current configuration of the panel
  ROS_INFO_STREAM("Saving configuration");
  rviz::Panel::save(config);
  config.mapSetValue("TextEntry", QString::fromStdString(std::string("test_field")));
}

void AddWayPoint::addPointFromUI(const tf::Transform point_pos)
{
  /*! Function for handling the signal of the RQT to add a new Way-Point in the RViz enviroment.
  */
  ROS_INFO("Point Added");
  makeArrow(point_pos, count);
  server->applyChanges();
}

void AddWayPoint::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  /*! This function is one of the most essential ones since it handles the events of the InteractiveMarkers from the
     User in the RViz Enviroment.
      In this function we have handlers for all the necessary events of the User Interaction with the
     InteractiveMarkers.
      When the user clicks on the User Interactive Arrow, it acts as button and adds new Way-Point in the RViz
     enviroment.
      When the User changes the pose of an InteractiveMarker from the RViz enviroment the position of the
     InteractiveMarker is updated synchronously in the RViz enviroment and in the RQT Widget.
      The Menu handlers take care of the User selected items from the menu of the Way-Point and call the necessary
     functions to change their state depending on the item selected from the menu.
  */
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

      Q_EMIT pointPoseUpdatedRViz(point_pos, feedback->marker_name.c_str());

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
        Q_EMIT pointDeleteRviz(marker_nr);
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

void AddWayPoint::pointPoseUpdated(const tf::Transform& point_pos, const char* marker_name)
{
  /*!
      @param point_pos takes the changed position of the InteractiveMarker either from RViz enviroment or the RQT.The
     vector for storing the Way-Points and the User Interaction Marker are updated according to the value of this
     parameter.
      @param marker_name is passed from this function either by taking information of the name of the Marker that has
     its position changed or by the RQT enviroment.

      Depending on the name of the Marker we either update the pose of the User Interactive Arrow or the Way-Point that
     is selected either from the RViz enviroment or the RQT Widget.
      In the case of updating a pose of a Way-Point, the corresponding position of the vector that stores all the poses
     for the Way-Points is updated as well.
  */
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

    if (index > waypoints_pos.size())
    {
      return;
    }

    waypoints_pos[index - 1] = point_pos;

    s << index;
    // Q_EMIT onUpdatePosCheckIkValidity(pose,index);
  }

  server->setPose(s.str(), pose);
  server->applyChanges();
}

visualization_msgs::Marker AddWayPoint::makeWayPoint(visualization_msgs::InteractiveMarker& msg)
{
  /*! Define a type and properties of a Marker for the Way-Point.
      This will be use as a base to define the shape, color and scale of the InteractiveMarker for the Way-Points.
  */
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale = WAY_POINT_SCALE_CONTROL;

  marker.color = WAY_POINT_COLOR;
  marker.action = visualization_msgs::Marker::ADD;
  return marker;
}

visualization_msgs::InteractiveMarkerControl& AddWayPoint::makeArrowControlDefault(visualization_msgs::InteractiveMarker& msg)
{
  visualization_msgs::InteractiveMarkerControl control_menu;
  control_menu.always_visible = true;

  control_menu.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;

  control_menu.name = "menu_select";
  msg.controls.push_back(control_menu);
  control_menu.markers.push_back(makeWayPoint(msg));

  visualization_msgs::InteractiveMarkerControl control_move3d;
  control_move3d.always_visible = true;

  control_move3d.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  control_move3d.name = "move";
  control_move3d.markers.push_back(makeWayPoint(msg));
  msg.controls.push_back(control_move3d);

  return msg.controls.back();
}

visualization_msgs::InteractiveMarkerControl& AddWayPoint::makeArrowControlDetails(visualization_msgs::InteractiveMarker& msg)
{
  visualization_msgs::InteractiveMarkerControl control_menu;
  control_menu.always_visible = true;

  control_menu.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control_menu.name = "menu_select";
  msg.controls.push_back(control_menu);
  control_menu.markers.push_back(makeWayPoint(msg));

  visualization_msgs::InteractiveMarkerControl control_view_details;
  control_view_details.always_visible = true;
  //*************rotate and move around the x-axis********************
  control_view_details.orientation.w = 1;
  control_view_details.orientation.x = 1;
  control_view_details.orientation.y = 0;
  control_view_details.orientation.z = 0;

  control_view_details.name = "rotate_x";
  control_view_details.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_view_details);

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

  control_view_details.name = "move_z";
  control_view_details.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_view_details);
  //*****************************************************************

  //*************rotate and move around the y-axis********************
  control_view_details.orientation.w = 1;
  control_view_details.orientation.x = 0;
  control_view_details.orientation.y = 0;
  control_view_details.orientation.z = 1;

  control_view_details.name = "rotate_y";
  control_view_details.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

  msg.controls.push_back(control_view_details);

  control_view_details.name = "move_y";
  control_view_details.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_view_details);
  control_view_details.markers.push_back(makeWayPoint(msg));

  msg.controls.push_back(control_view_details);

  //*****************************************************************
  return msg.controls.back();
}

void AddWayPoint::makeArrow(const tf::Transform& point_pos, int count_arrow)  //
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
    Q_EMIT addPointRViz(point_pos, count);
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
    Q_EMIT addPointRViz(point_pos, count);
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
  server->setCallback(int_marker.name, boost::bind(&AddWayPoint::processFeedback, this, _1));
  menu_handler.apply(*server, int_marker.name);
  // server->applyChanges();
  // Q_EMIT onUpdatePosCheckIkValidity(int_marker.pose,count_arrow);
}

void AddWayPoint::changeMarkerControlAndPose(std::string marker_name, bool set_control)
{
  /*! Handling the events from the clicked Menu Items for the Control of the Way-Point.
      Here the user can change the control either to freely move the Way-Point or get the 6DOF pose control option.
  */
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
  server->setCallback(int_marker.name, boost::bind(&AddWayPoint::processFeedback, this, _1));
  // Q_EMIT onUpdatePosCheckIkValidity(int_marker.pose,atoi(marker_name.c_str()));
}

void AddWayPoint::pointDeleted(std::string marker_name)
{
  /*! The point can be deleted either from the RViz or the RQT Widged.
      This function handles the event of removing a point from the RViz enviroment. It finds the name of the selected
     marker which the user wants to delete and updates the RViz enviroment and the vector that contains all the
     Way-Points.
   */
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

visualization_msgs::Marker AddWayPoint::makeInterArrow(visualization_msgs::InteractiveMarker& msg)
{
  /*! Define the Marker Arrow which the user can add new Way-Points with.

   */
  // define a marker
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::ARROW;
  marker.scale = ARROW_INTER_SCALE_CONTROL;

  // make the markers with interesting color
  marker.color = ARROW_INTER_COLOR;

  return marker;
}

visualization_msgs::InteractiveMarkerControl& AddWayPoint::makeInteractiveMarkerControl(visualization_msgs::InteractiveMarker& msg)
{
  /*! Set the User Interactive Marker with 6DOF control.
  */
  // //control for button interaction
  visualization_msgs::InteractiveMarkerControl control_button;
  control_button.always_visible = true;
  control_button.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control_button.name = "button_interaction";
  control_button.markers.push_back(makeInterArrow(msg));

  msg.controls.push_back(control_button);
  // server.reset( new interactive_markers::InteractiveMarkerServer("base_placement_plugin","",false));
  visualization_msgs::InteractiveMarkerControl control_inter_arrow;
  control_inter_arrow.always_visible = true;
  //*************rotate and move around the x-axis********************
  control_inter_arrow.orientation.w = 1;
  control_inter_arrow.orientation.x = 1;
  control_inter_arrow.orientation.y = 0;
  control_inter_arrow.orientation.z = 0;

  control_inter_arrow.name = "rotate_x";
  control_inter_arrow.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_inter_arrow);

  control_inter_arrow.name = "move_x";
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

  control_inter_arrow.name = "move_z";
  control_inter_arrow.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_inter_arrow);
  //*****************************************************************

  //*************rotate and move around the y-axis********************
  control_inter_arrow.orientation.w = 1;
  control_inter_arrow.orientation.x = 0;
  control_inter_arrow.orientation.y = 0;
  control_inter_arrow.orientation.z = 1;

  control_inter_arrow.name = "rotate_y";
  control_inter_arrow.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  msg.controls.push_back(control_inter_arrow);

  control_inter_arrow.name = "move_y";
  control_inter_arrow.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  msg.controls.push_back(control_inter_arrow);
  //*****************************************************************
  control_inter_arrow.markers.push_back(makeInterArrow(msg));

  return msg.controls.back();
}

void AddWayPoint::makeInteractiveMarker()
{
  /*! Create the User Interactive Marker and update the RViz enviroment.

  */
  visualization_msgs::InteractiveMarker inter_arrow_marker_;
  inter_arrow_marker_.header.frame_id = target_frame_;
  inter_arrow_marker_.scale = ARROW_INTERACTIVE_SCALE;

  ROS_INFO_STREAM("Marker Frame is:" << target_frame_);

  geometry_msgs::Pose pose;
  tf::poseTFToMsg(box_pos, inter_arrow_marker_.pose);

  inter_arrow_marker_.description = "Interaction Marker";

  // button like interactive marker. Detect when we have left click with the mouse and add new arrow then
  inter_arrow_marker_.name = "add_point_button";

  makeInteractiveMarkerControl(inter_arrow_marker_);
  server->insert(inter_arrow_marker_);
  // add interaction feedback to the markers
  server->setCallback(inter_arrow_marker_.name, boost::bind(&AddWayPoint::processFeedback, this, _1));
}
void AddWayPoint::parseWayPoints()
{
  /*! Get the vector of all Way-Points and convert it to geometry_msgs::Pose and send Qt signal when ready.
  */
  geometry_msgs::Pose target_pose;
  std::vector< geometry_msgs::Pose > waypoints;

  for (int i = 0; i < waypoints_pos.size(); i++)
  {
    tf::poseTFToMsg(waypoints_pos[i], target_pose);

    waypoints.push_back(target_pose);
  }

  Q_EMIT wayPoints_signal(waypoints);
}
void AddWayPoint::saveWayPointsToFile()
{
  /*! Function for saving all the Way-Points into yaml file.
      This function opens a Qt Dialog where the user can set the name of the Way-Points file and the location.
      Furthermore, it parses the way-points into a format that could be also loaded into the Plugin.
  */
  QString fileName =
      QFileDialog::getSaveFileName(this, tr("Save Way Points"), ".yaml", tr("Way Points (*.yaml);;All Files (*)"));

  if (fileName.isEmpty())
    return;
  else
  {
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly))
    {
      QMessageBox::information(this, tr("Unable to open file"), file.errorString());
      file.close();
      return;
    }

    YAML::Emitter out;
    out << YAML::BeginSeq;

    for (int i = 0; i < waypoints_pos.size(); i++)
    {
      out << YAML::BeginMap;
      std::vector< double > points_vec;
      points_vec.push_back(waypoints_pos[i].getOrigin().x());
      points_vec.push_back(waypoints_pos[i].getOrigin().y());
      points_vec.push_back(waypoints_pos[i].getOrigin().z());

      double rx, ry, rz;

      tf::Matrix3x3 m(waypoints_pos[i].getRotation());
      m.getRPY(rx, ry, rz, 1);
      points_vec.push_back(RAD2DEG(rx));
      points_vec.push_back(RAD2DEG(ry));
      points_vec.push_back(RAD2DEG(rz));

      out << YAML::Key << "name";
      out << YAML::Value << (i + 1);
      out << YAML::Key << "point";
      out << YAML::Value << YAML::Flow << points_vec;
      out << YAML::EndMap;
    }

    out << YAML::EndSeq;

    std::ofstream myfile;
    myfile.open(fileName.toStdString().c_str());
    myfile << out.c_str();
    myfile.close();
  }
}

void AddWayPoint::clearAllPointsRViz()
{
  waypoints_pos.clear();
  server->clear();
  // delete the waypoints_pos vector
  count = 0;
  makeInteractiveMarker();
  server->applyChanges();
}

void AddWayPoint::getRobotModelFrame_slot(const tf::Transform end_effector)
{
  /*! Set the frame of the all the InteractiveMarkers to correspond to the base of the loaded Robot Model.
      This function also initializes the count of the Way-Points and adds the User Interactive Marker to the scene and
     the RQT Widget.
  */

  target_frame_.assign("base_link");
  ROS_INFO_STREAM("The robot model frame is: " << target_frame_);

  box_pos = end_effector;

  clearAllPointsRViz();

  count = 0;
  makeInteractiveMarker();
  server->applyChanges();
}

}  // end of namespace for add_way_point

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(base_placement_plugin::AddWayPoint, rviz::Panel)
