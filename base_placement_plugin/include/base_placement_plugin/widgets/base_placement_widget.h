#ifndef base_placement_widget_H_
#define base_placement_widget_H_

#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string.h>

#include <ui_base_placement_widget.h>

#include <base_placement_plugin/add_way_point.h>
#include<base_placement_plugin/add_robot_base.h>


#include <QWidget>
#include <QTimer>
#include <QtConcurrentRun>
#include <QMainWindow>
#include <QTreeView>
#include <QStandardItemModel>
#include <QStandardItem>
#include <QSplitter>
#include <QHeaderView>
#include <QCompleter>
#include <QIntValidator>
#include <QDataStream>
#include <QString>
#include <QFileDialog>
#include <QMessageBox>
#include <QProgressBar>

// macros
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

namespace base_placement_plugin
{
namespace widgets
{
/*!
 *  \brief     Class for handling the User Interactions with the RQT Widget.
 *  \details   The PathPlanningWidget Class handles all User Interactions with the RQT GUI.
         This Class inherits from the QWidget superclass.
         The concept of the RQT widget is to add the same possabilities as the UI from the RViz enviroment and enabling
 simultanious communication betweet the RViz Enviroment and the RQT GUI.
 *  \author    Abhijit Makhal
 */

class BasePlacementWidget : public QWidget
{
  Q_OBJECT
public:
  //! RQT Widget Constructor.
  BasePlacementWidget(std::string ns = "");
  //! Virtual RQT Widget Destructor.
  virtual ~BasePlacementWidget();
  //! set the name of the RQT Widget.
  std::string get_name()
  {
    return "BasePlacementPlanner";
  }

protected:
  //! Widget Initialization.
  void init();
  std::string param_ns_;
  //! Protected variable for the Qt UI to access the Qt UI components.
  Ui::BasePlacementWidget ui_;
  //! Definition of an abstract data model.
  QStandardItemModel* pointDataModel;

  QObject* add_robot ;




  //! Bool value to show/unshow map
  bool show_union_map_;
  bool show_umodels_;
  std::string group_name_;

private:
  QStringList pointList;
  //! Checks the range of the points.
  void pointRange();
protected Q_SLOTS:
  //! Initialize the TreeView with the User Interactive Marker.
  void initTreeView();
  //! Handle the event of a Way-Point deleted from the RQT UI.
  void pointDeletedUI();
  //! Handle the event of a Way-Point added from the RQT UI.
  void pointAddUI();
  //! Insert a row in the TreeView.
  void insertRow(const tf::Transform& point_pos, const int count);
  //! Remove a row in the TreeView.
  void removeRow(int marker_nr);
  //! Handle the event when a User updates the pose of a Way-Point through the RQT UI.
  void pointPosUpdated_slot(const tf::Transform& point_pos, const char* marker_name);
  //! Get the selected Way-Point from the RQT UI.
  void selectedPoint(const QModelIndex& current, const QModelIndex& previous);
  //! Handle the even when the data in the TreeView has been changed.
  void treeViewDataChanged(const QModelIndex& index, const QModelIndex& index2);
  //! Slot for parsing the Way-Points and notifying the MoveIt.
  void parseWayPointBtn_slot();
  //! Send a signal that a save the Way-Points to a file button has been pressed.
  void savePointsToFile();
  //! Send a signal that a load the Way-Points from a file button has been pressed.
  void loadPointsFromFile();

  //! Send a signal that  a loading of reachability fiel is happening
  void loadReachabilityFile();
  //! Send a signal if the user wants to show the union map
  void showUnionMapFromUI();
  //! Send a signal to clear the union map
  void clearUnionMapFromUI();

  //! Send a signal if the user wants to start base placement by intution
  void startUserIntution();

  //!receive the base base waypoints and send them to place base
  void getWaypoints(std::vector<geometry_msgs::Pose> base_poses);




  //! Slot connected to a clear all points button click.
  void clearAllPoints_slot();
  //! Set the start pose of the User Interactive Marker to correspond to the loaded robot base frame.
  void setAddPointUIStartPos(const tf::Transform end_effector);
  //! Slot for disabling the TabWidged while Base Placement is executed.
  void PlaceBaseStartedHandler();
  //! Slot for enabling the TabWidged after Base Placement is executed.
  void PlaceBaseFinishedHandler();

  //! Sending base placement parametes
  void sendBasePlacementParamsFromUI();
  //! Set a message showing the process is completed
  void PlaceBaseCompleted_slot(double score);

  //! Set the Method name ComboBox
  void getBasePlacePlanMethod(std::vector< std::string > methods);
  //! Set the ouput type name in the ComboBox
  void getOutputType(std::vector< std::string > op_types);
  //! Set the ouput type name in the ComboBox
  void getRobotGroups(std::vector< std::string > groups);
  //! Get selected group from place_base
  void getSelectedGroup(std::string group_name);

  void selectedMethod(int index);
  void selectedOuputType(int op_index);
  void selectedRobotGroup(int index);
  void showUreachModels();

Q_SIGNALS:

  //! Signal to send reachability data after loading from file
  void reachabilityData_signal(std::multimap< std::vector< double >, std::vector< double > > PoseColFilter,
                               std::multimap< std::vector< double >, double > SphereCol, float res);
  // Signal bool to show union map
  void showUnionMap_signal(bool show_union_map_);
  // Signal bool to clear union map
  void clearUnionMap_signal(bool show_union_map_);

  //! Notify RViz enviroment that a new Way-Point has been added from RQT.
  void addPoint(const tf::Transform point_pos);
  //! Notify RViz enviroment that a new Way-Point has been deleted from RQT.
  void pointDelUI_signal(std::string marker_name);
  //! Notify RViz enviroment that a new Way-Point has been modified from RQT.
  void pointPosUpdated_signal(const tf::Transform& position, const char* marker_name);
  //! Signal to notify the base placement planner that the user has pressed the find base button
  void parseWayPointBtn_signal();
  //! Save to file button has been pressed.
  void saveToFileBtn_press();
  //! Signal that clear all points button has been pressed.
  void clearAllPoints_signal();
  //! Signal to send base placement parametes
  void basePlacementParamsFromUI_signal(int base_loc_size_, int high_socre_sp_);
  //! Sending selected method
  void SendSelectedMethod(int index);
  //! Sending selected ouput type
  void SendSelectedOpType(int op_index);
  //! Sending selected robot group
  void SendSelectedRobotGroup(int index);
  //! Sending selected umodel showing method
  void SendShowUmodel(bool umodel);
  //!Sending baseposes selected by user
  void SendBasePoses(std::vector<geometry_msgs::Pose>);


};
}

}  // end of namespace base_placement_plugin

#endif  // base_placement_widget_H_
