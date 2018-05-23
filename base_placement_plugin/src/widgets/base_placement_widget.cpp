#include <base_placement_plugin/widgets/base_placement_widget.h>
#include <base_placement_plugin/point_tree_model.h>
#include <base_placement_plugin/place_base.h>

#include <map_creator/hdf5_dataset.h>

#include <H5Cpp.h>
#include <hdf5.h>

namespace base_placement_plugin
{
namespace widgets
{
BasePlacementWidget::BasePlacementWidget(std::string ns) : param_ns_(ns)
{
  /*! Constructor which calls the init() function.

  */
  init();
  show_umodels_ = false;
}
BasePlacementWidget::~BasePlacementWidget()
{
}
void BasePlacementWidget::init()
{
  /*! Initializing the RQT UI. Setting up the default values for the UI components:
        - Default Values for the Base Placement Planner
        - Validators for the planners
        - Number of Base pose locations
        - Number of High Scoring Spheres

          .
        .
  */
  add_robot = 0;
  ui_.setupUi(this);

  ui_.lnEdit_BaseLocSize->setText("10");
  ui_.lnEdit_SpSize->setText("10");

  // ui_.lnEdit_BaseLocSize->setValidator(new QDoubleValidator(1,20,1,ui_.lnEdit_BaseLocSize));
  // QValidator *validator = new QIntValidator(1,100, this);
  // ui_.lnEdit_BaseLocSize->setValidator(validator);
  ui_.lnEdit_BaseLocSize->setValidator(new QIntValidator(1, 100, this));
  ui_.lnEdit_SpSize->setValidator(new QIntValidator(1, 20, this));

  // set progress bar when loading way-points from a yaml file. Could be nice when loading large way-points files
  ui_.progressBar->setRange(0, 100);
  ui_.progressBar->setValue(0);
  ui_.progressBar->hide();

  QStringList headers;
  headers << tr("Point") << tr("Position (m)") << tr("Orientation (deg)");
  PointTreeModel* model = new PointTreeModel(headers, "add_point_button");
  ui_.treeView->setModel(model);
  ui_.btn_LoadPath->setToolTip(tr("Load Way-Points from a file"));
  ui_.btn_SavePath->setToolTip(tr("Save Way-Points to a file"));
  ui_.btnAddPoint->setToolTip(tr("Add a new Way-Point"));
  ui_.btnRemovePoint->setToolTip(tr("Remove a selected Way-Point"));

  connect(ui_.btnAddPoint, SIGNAL(clicked()), this, SLOT(pointAddUI()));
  connect(ui_.btnRemovePoint, SIGNAL(clicked()), this, SLOT(pointDeletedUI()));
  connect(ui_.treeView->selectionModel(), SIGNAL(currentChanged(const QModelIndex&, const QModelIndex&)), this,
          SLOT(selectedPoint(const QModelIndex&, const QModelIndex&)));
  connect(ui_.treeView->selectionModel(), SIGNAL(currentChanged(const QModelIndex&, const QModelIndex&)), this,
          SLOT(treeViewDataChanged(const QModelIndex&, const QModelIndex&)));

  connect(ui_.targetPoint, SIGNAL(clicked()), this, SLOT(sendBasePlacementParamsFromUI()));
  connect(ui_.targetPoint, SIGNAL(clicked()), this, SLOT(parseWayPointBtn_slot()));

  connect(ui_.btn_LoadPath, SIGNAL(clicked()), this, SLOT(loadPointsFromFile()));

  connect(ui_.btn_LoadReachabilityFile, SIGNAL(clicked()), this, SLOT(loadReachabilityFile()));
  connect(ui_.btn_showUnionMap, SIGNAL(clicked()), this, SLOT(showUnionMapFromUI()));
  connect(ui_.btn_ClearUnionMap, SIGNAL(clicked()), this, SLOT(clearUnionMapFromUI()));

  connect(ui_.btn_SavePath, SIGNAL(clicked()), this, SLOT(savePointsToFile()));
  connect(ui_.btn_ClearAllPoints, SIGNAL(clicked()), this, SLOT(clearAllPoints_slot()));


  // connect(ui_.btn_moveToHome,SIGNAL(clicked()),this,SLOT(moveToHomeFromUI()));

  connect(ui_.combo_planGroup, SIGNAL(currentIndexChanged(int)), this, SLOT(selectedMethod(int)));
  connect(ui_.combo_opGroup, SIGNAL(currentIndexChanged(int)), this, SLOT(selectedOuputType(int)));
  connect(ui_.combo_robotModel, SIGNAL(currentIndexChanged(int)), this, SLOT(selectedRobotGroup(int)));
  connect(ui_.show_umodel_checkBox,SIGNAL(stateChanged(int)), this, SLOT(showUreachModels()));
}

void BasePlacementWidget::showUreachModels()
{
  if(show_umodels_)
    show_umodels_=false;
  else
    show_umodels_ = true;
  Q_EMIT SendShowUmodel(show_umodels_);
}

void BasePlacementWidget::getSelectedGroup(std::string group_name)
{
  group_name_ = group_name;
}


void BasePlacementWidget::showUnionMapFromUI()
{
  show_union_map_ = true;
  Q_EMIT showUnionMap_signal(show_union_map_);
}

void BasePlacementWidget::getRobotGroups(std::vector<std::string> groups)
{
  //ROS_INFO("Getting the robot groups");
  int robot_group = groups.size();

  disconnect(ui_.combo_robotModel, SIGNAL(currentIndexChanged(int)), this, SLOT(selectedRobotGroup(int)));
  ui_.combo_robotModel->clear();
  connect(ui_.combo_robotModel, SIGNAL(currentIndexChanged(int)), this, SLOT(selectedRobotGroup(int)));

  for(int i=0;i<robot_group;i++)
  {
    ui_.combo_robotModel->addItem(QString::fromStdString(groups[i]));
  }
}

void BasePlacementWidget::selectedRobotGroup(int index)
{
  Q_EMIT SendSelectedRobotGroup(index);
}

void BasePlacementWidget::getBasePlacePlanMethod(std::vector< std::string > methods)
{
  ROS_INFO("setting the name of the Method in combo box");
  for (int i = 0; i < methods.size(); i++)
  {
    ui_.combo_planGroup->addItem(QString::fromStdString(methods[i]));
  }
}

void BasePlacementWidget::selectedMethod(int index)
{
  Q_EMIT SendSelectedMethod(index);
  if(index ==4)
  {
    if(group_name_.size()>0)
    {
      add_robot = new AddRobotBase(0 , group_name_);
      ROS_INFO("AHA.The user intuition method is selected. Let's Play");
      //add_robot to widget
      connect(this, SIGNAL(parseWayPointBtn_signal()), add_robot, SLOT(parseWayPoints()));
      connect(add_robot, SIGNAL(baseWayPoints_signal(std::vector<geometry_msgs::Pose>)), this, SLOT(getWaypoints(std::vector<geometry_msgs::Pose>)));
      connect(this, SIGNAL(clearAllPoints_signal()), add_robot, SLOT(clearAllPointsRviz()));
    }
  }
  else
  {
    if(!add_robot == 0)
      delete add_robot;
    add_robot = 0;
  }
}

void BasePlacementWidget::getWaypoints(std::vector<geometry_msgs::Pose> base_poses)
{
  std::vector<geometry_msgs::Pose> new_base_poses;
  new_base_poses = base_poses;
  Q_EMIT SendBasePoses(new_base_poses);
}


void BasePlacementWidget::getOutputType(std::vector< std::string > op_types)
{
  ROS_INFO("setting the name of the output type in combo box");
  int op_type_size = op_types.size();

  for (int i = 0; i < op_type_size; i++)
  {
    ui_.combo_opGroup->addItem(QString::fromStdString(op_types[i]));
  }
}

void BasePlacementWidget::selectedOuputType(int index)
{
  Q_EMIT SendSelectedOpType(index);
}

void BasePlacementWidget::sendBasePlacementParamsFromUI()
{
  /*! This function takes care of sending the User Entered parameters from the RQT to the Base Placement Planner.
  */
  int base_loc_size_;
  int high_score_sp_;

  base_loc_size_ = ui_.lnEdit_BaseLocSize->text().toInt();
  high_score_sp_ = ui_.lnEdit_SpSize->text().toInt();

  Q_EMIT basePlacementParamsFromUI_signal(base_loc_size_, high_score_sp_);
}
void BasePlacementWidget::pointRange()
{
  /*! Get the current range of points from the TreeView.
      This is essential for setting up the number of the item that should be run next.
      Dealing with the data in the TreeView
  */
  QAbstractItemModel* model = ui_.treeView->model();
  int count = model->rowCount() - 1;
  ui_.txtPointName->setValidator(new QIntValidator(1, count, ui_.txtPointName));
}

void BasePlacementWidget::initTreeView()
{
  /*! Initialize the Qt TreeView and set the initial value of the User Interaction arrow.

  */
  QAbstractItemModel* model = ui_.treeView->model();

  model->setData(model->index(0, 0, QModelIndex()), QVariant("add_point_button"), Qt::EditRole);

  // update the validator for the lineEdit Point
  pointRange();
}
void BasePlacementWidget::selectedPoint(const QModelIndex& current, const QModelIndex& previous)
{
  /*! Get the selected point from the TreeView.
      This is used for updating the information of the lineEdit which informs gives the number of the currently selected
     Way-Point.
  */
  ROS_INFO_STREAM("Selected Index Changed" << current.row());

  if (current.parent() == QModelIndex())
    ui_.txtPointName->setText(QString::number(current.row()));
  else if ((current.parent() != QModelIndex()) && (current.parent().parent() == QModelIndex()))
    ui_.txtPointName->setText(QString::number(current.parent().row()));
  else
    ui_.txtPointName->setText(QString::number(current.parent().parent().row()));
}
void BasePlacementWidget::pointAddUI()
{
  /*! Function for adding new Way-Point from the RQT Widget.
  The user can set the position and orientation of the Way-Point by entering their values in the LineEdit fields.
  This function is connected to the AddPoint button click() signal and sends the addPoint(point_pos) to inform the RViz
  enviroment that a new Way-Point has been added.
  */
  double x, y, z, rx, ry, rz;
  x = ui_.LineEditX->text().toDouble();
  y = ui_.LineEditY->text().toDouble();
  z = ui_.LineEditZ->text().toDouble();
  rx = DEG2RAD(ui_.LineEditRx->text().toDouble());
  ry = DEG2RAD(ui_.LineEditRy->text().toDouble());
  rz = DEG2RAD(ui_.LineEditRz->text().toDouble());

  // // create transform
  tf::Transform point_pos(tf::Transform(tf::createQuaternionFromRPY(rx, ry, rz), tf::Vector3(x, y, z)));
  Q_EMIT addPoint(point_pos);

  pointRange();
}
void BasePlacementWidget::pointDeletedUI()
{
  /*! Function for deleting a Way-Point from the RQT GUI.
      The name of the Way-Point that needs to be deleted corresponds to the txtPointName line edit field.
      This slot is connected to the Remove Point button signal. After completion of this function a signal is send to
     Inform the RViz enviroment that a Way-Point has been deleted from the RQT Widget.
  */
  std::string marker_name;
  QString qtPointNr = ui_.txtPointName->text();
  marker_name = qtPointNr.toUtf8().constData();

  int marker_nr = atoi(marker_name.c_str());

  if (strcmp(marker_name.c_str(), "0") != 0)
  {
    removeRow(marker_nr);
    pointRange();
    Q_EMIT pointDelUI_signal(marker_name.c_str());
  }
}
void BasePlacementWidget::insertRow(const tf::Transform& point_pos, const int count)
{
  /*! Whenever we have a new Way-Point insereted either from the RViz or the RQT Widget the the TreeView needs to update
     the information and insert new row that corresponds to the new insered point.
      This function takes care of parsing the data recieved from the RViz or the RQT widget and creating new row with
     the appropriate data format and Children. One for the position giving us the current position of the Way-Point in
     all the axis.
      One child for the orientation giving us the Euler Angles of each axis.
  */

  ROS_INFO("inserting new row in the TreeView");
  QAbstractItemModel* model = ui_.treeView->model();

  // convert the quartenion to roll pitch yaw angle
  tf::Vector3 p = point_pos.getOrigin();
  tfScalar rx, ry, rz;
  point_pos.getBasis().getRPY(rx, ry, rz, 1);

  if (count == 0)
  {
    model->insertRow(count, model->index(count, 0));

    model->setData(model->index(0, 0, QModelIndex()), QVariant("add_point_button"), Qt::EditRole);
    pointRange();
  }
  else
  {
    if (!model->insertRow(count, model->index(count, 0)))  //&& count==0
    {
      return;
    }
    // set the strings of each axis of the position
    QString pos_x = QString::number(p.x());
    QString pos_y = QString::number(p.y());
    QString pos_z = QString::number(p.z());

    // repeat that with the orientation
    QString orient_x = QString::number(RAD2DEG(rx));
    QString orient_y = QString::number(RAD2DEG(ry));
    QString orient_z = QString::number(RAD2DEG(rz));

    model->setData(model->index(count, 0), QVariant(count), Qt::EditRole);

    // add a child to the last inserted item. First add children in the treeview that
    // are just telling the user that if he expands them he can see details about the position and orientation of each
    // point
    QModelIndex ind = model->index(count, 0);
    model->insertRows(0, 2, ind);
    QModelIndex chldind_pos = model->index(0, 0, ind);
    QModelIndex chldind_orient = model->index(1, 0, ind);
    model->setData(chldind_pos, QVariant("Position"), Qt::EditRole);
    model->setData(chldind_orient, QVariant("Orientation"), Qt::EditRole);
    //*****************************Set the children for the
    //position**********************************************************
    // now add information about each child separately. For the position we have coordinates for X,Y,Z axis.
    // therefore we add 3 rows of information
    model->insertRows(0, 3, chldind_pos);

    // next we set up the data for each of these columns. First the names
    model->setData(model->index(0, 0, chldind_pos), QVariant("X:"), Qt::EditRole);
    model->setData(model->index(1, 0, chldind_pos), QVariant("Y:"), Qt::EditRole);
    model->setData(model->index(2, 0, chldind_pos), QVariant("Z:"), Qt::EditRole);

    // second we add the current position information, for each position axis separately
    model->setData(model->index(0, 1, chldind_pos), QVariant(pos_x), Qt::EditRole);
    model->setData(model->index(1, 1, chldind_pos), QVariant(pos_y), Qt::EditRole);
    model->setData(model->index(2, 1, chldind_pos), QVariant(pos_z), Qt::EditRole);
    //***************************************************************************************************************************

    //*****************************Set the children for the
    //orientation**********************************************************
    // now we repeat everything again,similar as the position for adding the children for the orientation
    model->insertRows(0, 3, chldind_orient);
    // next we set up the data for each of these columns. First the names
    model->setData(model->index(0, 0, chldind_orient), QVariant("Rx:"), Qt::EditRole);
    model->setData(model->index(1, 0, chldind_orient), QVariant("Ry:"), Qt::EditRole);
    model->setData(model->index(2, 0, chldind_orient), QVariant("Rz:"), Qt::EditRole);

    // second we add the current position information, for each position axis separately
    model->setData(model->index(0, 2, chldind_orient), QVariant(orient_x), Qt::EditRole);
    model->setData(model->index(1, 2, chldind_orient), QVariant(orient_y), Qt::EditRole);
    model->setData(model->index(2, 2, chldind_orient), QVariant(orient_z), Qt::EditRole);
    //****************************************************************************************************************************
    pointRange();
  }
}
void BasePlacementWidget::removeRow(int marker_nr)
{
  /*! When the user deletes certain Way-Point either from the RViz or the RQT Widget the TreeView needs to delete that
   * particular row and update the state of the TreeWidget.
  */
  QAbstractItemModel* model = ui_.treeView->model();

  model->removeRow(marker_nr, QModelIndex());
  ROS_INFO_STREAM("deleting point nr: " << marker_nr);

  for (int i = marker_nr; i <= model->rowCount(); ++i)
  {
    model->setData(model->index((i - 1), 0, QModelIndex()), QVariant((i - 1)), Qt::EditRole);
  }
  // check how to properly set the selection
  ui_.treeView->selectionModel()->setCurrentIndex(model->index((model->rowCount() - 1), 0, QModelIndex()),
                                                  QItemSelectionModel::ClearAndSelect);
  ui_.txtPointName->setText(QString::number(model->rowCount() - 1));
  pointRange();
}

void BasePlacementWidget::pointPosUpdated_slot(const tf::Transform& point_pos, const char* marker_name)
{
  /*! When the user updates the position of the Way-Point or the User Interactive Marker, the information in the
     TreeView also needs to be updated to correspond to the current pose of the InteractiveMarkers.

  */
  QAbstractItemModel* model = ui_.treeView->model();

  tf::Vector3 p = point_pos.getOrigin();
  tfScalar rx, ry, rz;
  point_pos.getBasis().getRPY(rx, ry, rz, 1);

  rx = RAD2DEG(rx);
  ry = RAD2DEG(ry);
  rz = RAD2DEG(rz);

  // set the strings of each axis of the position
  QString pos_x = QString::number(p.x());
  QString pos_y = QString::number(p.y());
  QString pos_z = QString::number(p.z());

  // repeat that with the orientation
  QString orient_x = QString::number(rx);
  QString orient_y = QString::number(ry);
  QString orient_z = QString::number(rz);

  if ((strcmp(marker_name, "add_point_button") == 0) || (atoi(marker_name) == 0))
  {
    QString pos_s;
    pos_s = pos_x + "; " + pos_y + "; " + pos_z + ";";
    QString orient_s;
    orient_s = orient_x + "; " + orient_y + "; " + orient_z + ";";

    model->setData(model->index(0, 0), QVariant("add_point_button"), Qt::EditRole);
    model->setData(model->index(0, 1), QVariant(pos_s), Qt::EditRole);
    model->setData(model->index(0, 2), QVariant(orient_s), Qt::EditRole);
  }
  else
  {
    int changed_marker = atoi(marker_name);
    //**********************update the positions and orientations of the children as
    //well***********************************************************************************************
    QModelIndex ind = model->index(changed_marker, 0);
    QModelIndex chldind_pos = model->index(0, 0, ind);
    QModelIndex chldind_orient = model->index(1, 0, ind);

    // second we add the current position information, for each position axis separately
    model->setData(model->index(0, 1, chldind_pos), QVariant(pos_x), Qt::EditRole);
    model->setData(model->index(1, 1, chldind_pos), QVariant(pos_y), Qt::EditRole);
    model->setData(model->index(2, 1, chldind_pos), QVariant(pos_z), Qt::EditRole);

    // second we add the current position information, for each position axis separately
    model->setData(model->index(0, 2, chldind_orient), QVariant(orient_x), Qt::EditRole);
    model->setData(model->index(1, 2, chldind_orient), QVariant(orient_y), Qt::EditRole);
    model->setData(model->index(2, 2, chldind_orient), QVariant(orient_z), Qt::EditRole);
    //*****************************************************************************************************************************************************************************************
  }
}

void BasePlacementWidget::treeViewDataChanged(const QModelIndex& index, const QModelIndex& index2)
{
  /*! This function handles the user interactions in the TreeView Widget.
      The function captures an event of data change and updates the information in the TreeView and the RViz enviroment.
  */
  qRegisterMetaType< std::string >("std::string");
  QAbstractItemModel* model = ui_.treeView->model();
  QVariant index_data;
  ROS_INFO_STREAM("Data changed in index:" << index.row() << "parent row" << index2.parent().row());

  if ((index.parent() == QModelIndex()) && (index.row() != 0))
  {
  }
  else if (((index.parent().parent()) != QModelIndex()) && (index.parent().parent().row() != 0))
  {
    QModelIndex main_root = index.parent().parent();
    std::stringstream s;
    s << main_root.row();
    std::string temp_str = s.str();

    QModelIndex chldind_pos = model->index(0, 0, main_root.sibling(main_root.row(), 0));
    QModelIndex chldind_orient = model->index(1, 0, main_root.sibling(main_root.row(), 0));

    QVariant pos_x = model->data(model->index(0, 1, chldind_pos), Qt::EditRole);
    QVariant pos_y = model->data(model->index(1, 1, chldind_pos), Qt::EditRole);
    QVariant pos_z = model->data(model->index(2, 1, chldind_pos), Qt::EditRole);

    QVariant orient_x = model->data(model->index(0, 2, chldind_orient), Qt::EditRole);
    QVariant orient_y = model->data(model->index(1, 2, chldind_orient), Qt::EditRole);
    QVariant orient_z = model->data(model->index(2, 2, chldind_orient), Qt::EditRole);

    tf::Vector3 p(pos_x.toDouble(), pos_y.toDouble(), pos_z.toDouble());

    tfScalar rx, ry, rz;
    rx = DEG2RAD(orient_x.toDouble());
    ry = DEG2RAD(orient_y.toDouble());
    rz = DEG2RAD(orient_z.toDouble());

    tf::Transform point_pos = tf::Transform(tf::createQuaternionFromRPY(rx, ry, rz), p);

    Q_EMIT pointPosUpdated_signal(point_pos, temp_str.c_str());
  }
}
void BasePlacementWidget::parseWayPointBtn_slot()
{
  /*! Letting know the Base Placement Planner that the user has pressed the find base button
  */
  Q_EMIT parseWayPointBtn_signal();
}

void BasePlacementWidget::loadPointsFromFile()
{
  /*! Slot that takes care of opening a previously saved Way-Points yaml file.
      Opens Qt Dialog for selecting the file, opens the file and parses the data.
      After reading and parsing the data from the file, the information regarding the pose of the Way-Points is send to
     the RQT and the RViz so they can update their enviroments.
  */
  QString fileName =
      QFileDialog::getOpenFileName(this, tr("Open Way Points File"), "", tr("Way Points (*.yaml);;All Files (*)"));

  if (fileName.isEmpty())
  {
    ui_.tabWidget->setEnabled(true);
    ui_.progressBar->hide();
    return;
  }
  else
  {
    ui_.tabWidget->setEnabled(false);
    ui_.progressBar->show();
    QFile file(fileName);

    if (!file.open(QIODevice::ReadOnly))
    {
      QMessageBox::information(this, tr("Unable to open file"), file.errorString());
      file.close();
      ui_.tabWidget->setEnabled(true);
      ui_.progressBar->hide();
      return;
    }
    // clear all the scene before loading all the new points from the file!!
    clearAllPoints_slot();

    ROS_INFO_STREAM("Opening the file: " << fileName.toStdString());
    std::string fin(fileName.toStdString());

    YAML::Node doc;
    doc = YAML::LoadFile(fin);
    // define double for percent of completion
    double percent_complete;
    int end_of_doc = doc.size();

    for (size_t i = 0; i < end_of_doc; i++)
    {
      std::string name;
      geometry_msgs::Pose pose;
      tf::Transform pose_tf;

      double x, y, z, rx, ry, rz;
      name = doc[i]["name"].as< std::string >();
      x = doc[i]["point"][0].as< double >();
      y = doc[i]["point"][1].as< double >();
      z = doc[i]["point"][2].as< double >();
      rx = doc[i]["point"][3].as< double >();
      ry = doc[i]["point"][4].as< double >();
      rz = doc[i]["point"][5].as< double >();

      rx = DEG2RAD(rx);
      ry = DEG2RAD(ry);
      rz = DEG2RAD(rz);

      pose_tf = tf::Transform(tf::createQuaternionFromRPY(rx, ry, rz), tf::Vector3(x, y, z));

      percent_complete = (i + 1) * 100 / end_of_doc;
      ui_.progressBar->setValue(percent_complete);
      Q_EMIT addPoint(pose_tf);
    }
    ui_.tabWidget->setEnabled(true);
    ui_.progressBar->hide();
  }
}
void BasePlacementWidget::savePointsToFile()
{
  /*! Just inform the RViz enviroment that Save Way-Points button has been pressed.
   */
  Q_EMIT saveToFileBtn_press();
}
void BasePlacementWidget::clearAllPoints_slot()
{
  /*! Clear all the Way-Points from the RViz enviroment and the TreeView.
  */
  QAbstractItemModel* model = ui_.treeView->model();
  model->removeRows(0, model->rowCount());
  ui_.txtPointName->setText("0");
  tf::Transform t;
  t.setIdentity();
  insertRow(t, 0);
  pointRange();

  Q_EMIT clearAllPoints_signal();
}
void BasePlacementWidget::setAddPointUIStartPos(const tf::Transform end_effector)
{
  /*! Setting the default values for the Add New Way-Point from the RQT.
      The information is taken to correspond to the pose of the loaded Robot end-effector.
  */
  tf::Vector3 p = end_effector.getOrigin();
  tfScalar rx, ry, rz;
  end_effector.getBasis().getRPY(rx, ry, rz, 1);

  rx = RAD2DEG(rx);
  ry = RAD2DEG(ry);
  rz = RAD2DEG(rz);

  // set up the starting values for the lineEdit of the positions
  ui_.LineEditX->setText(QString::number(p.x()));
  ui_.LineEditY->setText(QString::number(p.y()));
  ui_.LineEditZ->setText(QString::number(p.z()));
  // set up the starting values for the lineEdit of the orientations, in Euler angles
  ui_.LineEditRx->setText(QString::number(rx));
  ui_.LineEditRy->setText(QString::number(ry));
  ui_.LineEditRz->setText(QString::number(rz));

  // set up the starting values for the lineEdit of the positions
  ui_.LineEditX->setText(QString::number(p.x()));
  ui_.LineEditY->setText(QString::number(p.y()));
  ui_.LineEditZ->setText(QString::number(p.z()));
  // set up the starting values for the lineEdit of the orientations, in Euler angles
  ui_.LineEditRx->setText(QString::number(rx));
  ui_.LineEditRy->setText(QString::number(ry));
  ui_.LineEditRz->setText(QString::number(rz));
}

void BasePlacementWidget::PlaceBaseStartedHandler()
{
  /*! Disable the RQT Widget when the Base Placement is executing.
  */
  ui_.tabWidget->setEnabled(false);
  ui_.targetPoint->setEnabled(false);
}
void BasePlacementWidget::PlaceBaseFinishedHandler()
{
  /*! Enable the RQT Widget when the Base Placement execution is completed.
  */
  ui_.tabWidget->setEnabled(true);
  ui_.targetPoint->setEnabled(true);
}
void BasePlacementWidget::PlaceBaseCompleted_slot(double score)
{
  /* A message showing task has completed */

  ui_.lbl_placeBaseCompleted->setText("COMPLETED. Score: " + QString::number(score));
}

void BasePlacementWidget::loadReachabilityFile()
{
  /*! Slot that takes care of opening and loading data from an inverse Reachability Map file
  */
  QString fileName = QFileDialog::getOpenFileName(this, tr("Open Inverse Reachability File"), "",
                                                  tr("Reachability (*.h5);;All Files (*)"));

  if (fileName.isEmpty())
  {
    ui_.tabWidget->setEnabled(true);
    ui_.progressBar->hide();
    return;
  }
  else
  {
    ui_.tabWidget->setEnabled(false);
    ui_.progressBar->show();
    QFile fileN(fileName);

    if (!fileN.open(QIODevice::ReadOnly))
    {
      QMessageBox::information(this, tr("Unable to open file"), fileN.errorString());
      fileN.close();
      ui_.tabWidget->setEnabled(true);
      ui_.progressBar->hide();
      return;
    }
    // clear all the scene before loading all the new points from the file!!
    // clearAllPoints_slot();

    ROS_INFO_STREAM("Opening the file: " << fileName.toStdString());

    std::string fileh5 = fileName.toStdString();
    const char* FILE = fileh5.c_str();

    MultiMap pose_col_filter;
    MapVecDouble sp;
    float res;

    hdf5_dataset::Hdf5Dataset h5file(FILE);
    h5file.open();
    h5file.loadMapsFromDataset(pose_col_filter, sp, res);


    std::multimap< std::vector< double >, double > sphere_col;
    for(MapVecDouble::iterator it= sp.begin(); it!=sp.end();++it)
    {
      std::vector<double> sphere_coord(3);
      sphere_coord[0] = it->first[0];
      sphere_coord[1] = it->first[1];
      sphere_coord[2] = it->first[2];
      sphere_col.insert(std::pair<std::vector<double>, double> (sphere_coord, it->second));
    }

    //Just checking
   // ROS_INFO("Size of poses dataset: %lu", PoseColFilter.size());
   //ROS_INFO("Size of Sphere dataset: %lu", SphereCol.size());

    Q_EMIT reachabilityData_signal(pose_col_filter, sphere_col, res);
  }
  ui_.tabWidget->setEnabled(true);
  ui_.progressBar->hide();
}



void BasePlacementWidget::clearUnionMapFromUI()
{
  show_union_map_ = false;
  Q_EMIT clearUnionMap_signal(show_union_map_);
}
/*void BasePlacementWidget::moveToHomeFromUI()
{
  Q_EMIT moveToHomeFromUI_signal();
}*/
}
}
