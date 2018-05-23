#ifndef ADD_ROBOT_BASE_H
#define ADD_ROBOT_BASE_H
#ifndef Q_MOC_RUN
#include<stdio.h>
#include<iostream>
#include <string.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <iterator>

#include <rviz/panel.h>
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <rviz/default_plugin/interactive_markers/interactive_marker.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Scalar.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/string_property.h>
#include <base_placement_plugin/widgets/base_placement_widget.h>
#include <base_placement_plugin/place_base.h>
#include <base_placement_plugin/create_marker.h>

#include <QWidget>
#include <QCursor>
#include <QObject>
#include <QKeyEvent>
#include <QHBoxLayout>
#include <QTimer>
#include <QtConcurrentRun>
#include <QFuture>

#endif


namespace rviz
{
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
class StringProperty;
class BoolProperty;
}


class AddRobotBase : public QObject
{
    Q_OBJECT;
  public:
    AddRobotBase(QWidget* parent, std::string group_name);
    virtual ~AddRobotBase();
    void init();

    virtual void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

    void changeMarkerControlAndPose(std::string marker_name, bool set_control);
    visualization_msgs::InteractiveMarkerControl& makeArrowControlDefault(visualization_msgs::InteractiveMarker& msg);
    visualization_msgs::InteractiveMarkerControl& makeArrowControlDetails(visualization_msgs::InteractiveMarker& msg);
    visualization_msgs::Marker makeWayPoint(visualization_msgs::InteractiveMarker& msg);
    void pointDeleted(std::string marker_name);
    void makeArrow(const tf::Transform& point_pos, int count_arrow);
    void makeInteractiveMarker();
    visualization_msgs::InteractiveMarkerControl& makeInteractiveMarkerControl(visualization_msgs::InteractiveMarker& msg);
    visualization_msgs::Marker makeInterArrow(visualization_msgs::InteractiveMarker& msg);
    visualization_msgs::MarkerArray makeRobotMarker(visualization_msgs::InteractiveMarker& msg,  bool waypoint);


    void pointPoseUpdated(const tf::Transform& point_pos, const char* marker_name);
    void getWaypoints(std::vector<geometry_msgs::Pose>& waypoints);



    public Q_SLOTS:
    void parseWayPoints();
    void clearAllPointsRviz();
    void getRobotModelFrame_slot(const tf::Transform end_effector);

protected:
    QWidget* widget_;

    Q_SIGNALS:
    void baseWayPoints_signal(std::vector<geometry_msgs::Pose> base_poses);

private:
    //! Define a server for the Interactive Markers.
      boost::shared_ptr< interactive_markers::InteractiveMarkerServer > server;
      interactive_markers::MenuHandler menu_handler;

      //! for arrows...........need to be modified
      std_msgs::ColorRGBA WAY_POINT_COLOR;
      std_msgs::ColorRGBA ROBOT_WAY_POINT_COLOR;
      std_msgs::ColorRGBA ARROW_INTER_COLOR;
      std_msgs::ColorRGBA ROBOT_INTER_COLOR;
      geometry_msgs::Vector3 WAY_POINT_SCALE_CONTROL;
      geometry_msgs::Vector3 ARROW_INTER_SCALE_CONTROL;

      float INTERACTIVE_MARKER_SCALE;
      float ARROW_INTERACTIVE_SCALE;

      //! Vector for storing all the User Entered Way-Points.
      std::vector< tf::Transform > waypoints_pos;
      //! The position of the User handlable Interactive Marker.
      tf::Transform box_pos;
      int count;
      std::string target_frame_;

      visualization_msgs::MarkerArray robot_markers_;
      CreateMarker* mark_;
      std::string group_name_;

};






#endif // ADD_ROBOT_BASE_H
