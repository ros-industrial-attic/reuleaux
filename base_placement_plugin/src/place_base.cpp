#include <boost/function.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>

#include <base_placement_plugin/place_base.h>

#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

#include <map_creator/WorkSpace.h>
#include <map_creator/sphere_discretization.h>
#include <map_creator/kinematics.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

using namespace std;
using namespace octomap;
using namespace octomath;
using namespace sphere_discretization;
using namespace kinematics;

PlaceBase::PlaceBase(QObject *parent)
{
  init();
}
PlaceBase::~PlaceBase()
{
}

void PlaceBase::init()
{
}

void PlaceBase::BasePlacementHandler(std::vector< geometry_msgs::Pose > waypoints)
{
  /*! The function for base placement has been placed in a separate thread.
      This prevents the RViz and the Plugin to lock.
  */
  ROS_INFO("Starting concurrent process for Base Placement");
  QFuture< void > future = QtConcurrent::run(this, &PlaceBase::findbase, waypoints);
}

void PlaceBase::initRvizDone()
{
  /*!
      Once the initialization of the RViz is has finished, this function sends the pose of the initial marker. It also
     sends the method list and the visualization method lists.
  */
  ROS_INFO("RViz is done now we need to emit the signal");

  tf::Vector3 vec(0, 0, 0);
  tf::Quaternion quat(0, 0, 0, 1);
  quat.normalize();
  tf::Transform trns;
  trns.setOrigin(vec);
  trns.setRotation(quat);

  // Q_EMIT getRobotModelFrame_signal(trns);
  Q_EMIT getinitialmarkerFrame_signal(trns);

  ROS_INFO("Sending the method list");
  method_names_.push_back("PrincipalComponentAnalysis");
  method_names_.push_back("GraspReachabilityScore");
  method_names_.push_back("IKSolutionScore");
  Q_EMIT sendBasePlaceMethods_signal(method_names_);

  ROS_INFO("Sending the output visualization method list");
  output_type_.push_back("Arrows");
  output_type_.push_back("RobotModel");
  Q_EMIT sendOuputType_signal(output_type_);
}

void PlaceBase::getSelectedMethod(int index)
{
  /* The selected method signal has been received
  */
  selected_method_ = index;
  ROS_INFO_STREAM("selected_method: " << method_names_[index]);
}

void PlaceBase::getSelectedOpType(int op_index)
{
  /* The selected output type signal has been received
  */
  selected_op_type_ = op_index;
  ROS_INFO_STREAM("selected visualization method: " << output_type_[op_index]);
}

void PlaceBase::setBasePlaceParams(int base_loc_size, int high_score_sp)
{
  /*! Set the necessary parameters for the Base Placement process.
      These parameters correspond to the ones that the user has entered or the default ones before pressing the find
     base button.
      The base loc size is the number of ouput the user desires
      The highscoresp is the number of high scoring spheres we are collecting all the valid poses
  */
  ROS_INFO_STREAM("Base Placement parameters from UI:\n Number of base locations:"
                  << base_loc_size << "\n Number of HighScore Spheres:" << high_score_sp);

  BASE_LOC_SIZE_ = base_loc_size;
  HIGH_SCORE_SP_ = high_score_sp;
  if (BASE_LOC_SIZE_ <= 0)
  {
    ROS_ERROR_STREAM("Please provide a valid number of how many base locations do you need.");
  }

  if (HIGH_SCORE_SP_ <= 0)
  {
    ROS_ERROR_STREAM("Please provide a valid number of how many spheres do you need to create valid base poses");
  }
}

void PlaceBase::findbase(std::vector< geometry_msgs::Pose > grasp_poses)
{
  /*! The main function that finds the base. First it transforms all the poses form the loaded inverse reachability
   * file with the grasp poses selected by the user. Then by nearest neighbor searching it associates all the poses
   * to the corresponding spheres. By normalization it decides the color of the spheres. Then from the highest scoring
   * spheres it calls the desired methods for finding the final base locations.
  */

  Q_EMIT basePlacementProcessStarted();

  if (grasp_poses.size() == 0)
    ROS_INFO("Please provide atleast one grasp pose.");

  else
  {
    if (PoseColFilter.size() == 0)
    {
      ROS_INFO("No Inverse Reachability Map found. Please provide an Inverse Reachability map.");
    }
    else
    {
      SphereDiscretization sd;
      baseTrnsCol.clear();
      sphereColor.clear();
      highScoreSp.clear();
      final_base_poses.clear();
      GRASP_POSES_ = grasp_poses;
      sd.associatePose(baseTrnsCol, grasp_poses, PoseColFilter, res);
      ROS_INFO("Size of baseTrnsCol dataset: %lu", baseTrnsCol.size());

      // Normalization for inverse Reachability Index

      vector< int > poseCount;
      for (multimap< vector< double >, vector< double > >::iterator it = baseTrnsCol.begin(); it != baseTrnsCol.end();
           ++it)
      {
        int num = baseTrnsCol.count(it->first);
        poseCount.push_back(num);
      }

      vector< int >::const_iterator it;
      it = max_element(poseCount.begin(), poseCount.end());
      int max_number = *it;
      // cout<<"Maximum number of poses in a voxel: "<<max_number<<endl;

      it = min_element(poseCount.begin(), poseCount.end());
      int min_number = *it;
      // cout<<"Minimum number of poses in a voxel: "<<min_number<<endl;

      // Colors of spheres are determined by index
      for (multimap< vector< double >, vector< double > >::iterator it = baseTrnsCol.begin(); it != baseTrnsCol.end();
           ++it)
      {
        float d = ((float(baseTrnsCol.count(it->first)) - min_number) / (max_number - min_number)) * 100;
        if (d > 1)
        {
          sphereColor.insert(pair< vector< double >, double >(it->first, double(d)));
        }
      }
      ROS_INFO("Union map has been created. Can now visualize Union Map.");
      ROS_INFO("Poses in Union Map: %lu", baseTrnsCol.size());
      ROS_INFO("Spheres in Union Map: %lu", sphereColor.size());

      multiset< pair< double, vector< double > > > scoreWithSp;
      for (map< vector< double >, double >::iterator it = sphereColor.begin(); it != sphereColor.end(); ++it)
      {
        scoreWithSp.insert(pair< double, vector< double > >(it->second, it->first));
      }
      // ROS_INFO("Numer of Spheres : %lu",scoreWithSp.size());

      for (multiset< pair< double, vector< double > > >::reverse_iterator it = scoreWithSp.rbegin();
           it != scoreWithSp.rend(); ++it)
      {
        highScoreSp.push_back(it->second);
      }
      ROS_INFO("Size of high score sp: %lu", highScoreSp.size());  // filtered sp centers based on their scores

      BasePlaceMethodHandler();

      for (int i = 0; i < final_base_poses.size(); ++i)
      {
        tf2::Quaternion quat(final_base_poses[i].orientation.x, final_base_poses[i].orientation.y,
                             final_base_poses[i].orientation.z, final_base_poses[i].orientation.w);
        tf2::Matrix3x3 m(quat);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_INFO("Optimal base pose[%d]: Position: %f, %f, %f, Orientation: %f, %f, %f", i + 1,
                 final_base_poses[i].position.x, final_base_poses[i].position.y, final_base_poses[i].position.z, roll,
                 pitch, yaw);
      }

      // showBaseLocations(final_base_poses);
      OuputputVizHandler(final_base_poses);
    }
  }

  Q_EMIT basePlacementProcessCompleted();

  Q_EMIT basePlacementProcessFinished();
  ROS_INFO("FindBase Task Finished");
}

void PlaceBase::BasePlaceMethodHandler()
{
  /* Switch cases for selecting method for base placement
  */
  switch (selected_method_)
  {
    case 0:
    {
      findBaseByPCA();
      break;
    }

    case 1:
    {
      findBaseByGraspReachabilityScore();
      break;
    }

    case 2:
    {
      findBaseByIKSolutionScore();
      ;
      break;
    }
  }
}

void PlaceBase::OuputputVizHandler(vector< geometry_msgs::Pose > po)
{
  /* Switch cases for selecting output type for visualization
  */
  switch (selected_op_type_)
  {
    case 0:
    {
      showBaseLocationsbyArrow(po);
      break;
    }

    case 1:
    {
      showBaseLocationsbyRobotModel(po);
      break;
    }
  }
}

void PlaceBase::findBaseByPCA()
{
  /* PCA Method: The planner takes desired number of high scoring spheres and implements PCA for finding optimal
   * orientations from all the poses correspond to that sphere. One pose from one sphere.
  */
  ROS_INFO("Finding optimal base pose by PCA.");
  SphereDiscretization sd;

  vector< geometry_msgs::Pose > probBasePoses;
  map_creator::WorkSpace ws;
  for (int i = 0; i < BASE_LOC_SIZE_; ++i)
  {
    map_creator::WsSphere wss;
    wss.point.x = highScoreSp[i][0];
    wss.point.y = highScoreSp[i][1];
    wss.point.z = highScoreSp[i][2];
    vector< double > basePose;
    basePose.push_back(highScoreSp[i][0]);
    basePose.push_back(highScoreSp[i][1]);
    basePose.push_back(highScoreSp[i][2]);
    multimap< vector< double >, vector< double > >::iterator it;
    for (it = baseTrnsCol.lower_bound(basePose); it != baseTrnsCol.upper_bound(basePose); ++it)
    {
      geometry_msgs::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      wss.poses.push_back(pp);
    }
    ws.WsSpheres.push_back(wss);
  }

  for (int i = 0; i < ws.WsSpheres.size(); ++i)
  {
    geometry_msgs::Pose final_base_pose;
    //sd.findOptimalPosebyAverage(ws.WsSpheres[i].poses, final_base_pose);  // Calling the PCA

    sd.findOptimalPosebyPCA(ws.WsSpheres[i].poses, final_base_pose);  // Calling the PCA
    final_base_pose.position.x = ws.WsSpheres[i].point.x;
    final_base_pose.position.y = ws.WsSpheres[i].point.y;
    final_base_pose.position.z = ws.WsSpheres[i].point.z;
    final_base_poses.push_back(final_base_pose);
  }
}

void PlaceBase::findBaseByGraspReachabilityScore()
{
  /* GraspReachabilityScore Method: The planner takes desired number of high scoring spheres and collects all the poses
     from them. Then calculates reachability of that poses with all the grasp points. The poses that can reach all the
     grasp poses can be considered as optimal base locations.

  */
  ROS_INFO("Finding optimal base pose by GraspReachabilityScore.");
  SphereDiscretization sd;
  Kinematics k;

  vector< geometry_msgs::Pose > probBasePoses;

  int numofSp = HIGH_SCORE_SP_;  // From how many spheres we are collecting all the poses
  for (int i = 0; i < numofSp; ++i)
  {
    multimap< vector< double >, vector< double > >::iterator it;
    for (it = baseTrnsCol.lower_bound(highScoreSp[i]); it != baseTrnsCol.upper_bound(highScoreSp[i]); ++it)
    {
      geometry_msgs::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      probBasePoses.push_back(pp);
    }
  }

  ROS_INFO_STREAM("Size of Probable Base poses: " << probBasePoses.size() << " with Spheres: " << numofSp);

  multiset< pair< int, vector< double > > > basePoseWithHits;
  for (int i = 0; i < probBasePoses.size(); ++i)
  {
    int numofHits = 0;
    for (int j = 0; j < GRASP_POSES_.size(); ++j)
    {
      int nsolns = 0;
      numofHits += k.isIkSuccesswithTransformedBase(probBasePoses[i], GRASP_POSES_[j], nsolns);
    }
    vector< double > baseP;
    sd.convertPoseToVector(probBasePoses[i], baseP);
    basePoseWithHits.insert(pair< int, vector< double > >(numofHits, baseP));
  }
  vector< geometry_msgs::Pose > final_base_loc;
  for (multiset< pair< int, vector< double > > >::iterator it = basePoseWithHits.begin(); it != basePoseWithHits.end();
       ++it)
  {
    if ((it->first) >= GRASP_POSES_.size())
    {
      geometry_msgs::Pose final_base;
      sd.convertVectorToPose(it->second, final_base);
      final_base_loc.push_back(final_base);
    }
  }
  ROS_INFO_STREAM("Poses that have optimal base location: " << final_base_loc.size());
  if (final_base_loc.size() < BASE_LOC_SIZE_)
    ROS_ERROR_STREAM("The map you have provided is not suitable for base placement. Please provide a valid map.");
  for (int i = 0; i < BASE_LOC_SIZE_; ++i)
  {
    final_base_poses.push_back(final_base_loc[i]);
  }
}

void PlaceBase::findBaseByIKSolutionScore()
{
  /* IKSolutionScore Method: The planner takes desired number of high scoring spheres and collects all the poses from
     them. Then calculates number of Ik solutions of that poses with all the grasp points. The poses that have the
     highest score can be considered as optimal base locations.

  */
  ROS_INFO("Finding optimal base pose by GraspReachabilityScore.");
  SphereDiscretization sd;
  Kinematics k;

  vector< geometry_msgs::Pose > probBasePoses;
  int numofSp = HIGH_SCORE_SP_;  // From how many spheres we are collecting all the poses
  for (int i = 0; i < numofSp; ++i)
  {
    multimap< vector< double >, vector< double > >::iterator it;
    for (it = baseTrnsCol.lower_bound(highScoreSp[i]); it != baseTrnsCol.upper_bound(highScoreSp[i]); ++it)
    {
      geometry_msgs::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      probBasePoses.push_back(pp);
    }
  }

  ROS_INFO("Size of Probable Base poses: %lu with %d Spheres", probBasePoses.size(), numofSp);
  // ROS_INFO_STREAM("Size of Probable Base poses: "<<probBasePoses.size());
  // ROS_INFO_STREAM(" with Spheres: "<<numofSp);
  multiset< pair< double, vector< double > > > basePoseWithHits;
  int max_solns = GRASP_POSES_.size() * 8;
  int min_solns = 0;
  for (int i = 0; i < probBasePoses.size(); ++i)
  {
    int numofHits = 0;
    int solns = 0;
    for (int j = 0; j < GRASP_POSES_.size(); ++j)
    {
      int nsolns = 0;
      numofHits += k.isIkSuccesswithTransformedBase(probBasePoses[i], GRASP_POSES_[j], nsolns);
      solns += nsolns;
    }
    vector< double > baseP;
    sd.convertPoseToVector(probBasePoses[i], baseP);
    double basePlaceScore = (double(solns) - double(min_solns)) / (double(max_solns) - double(min_solns));
    basePoseWithHits.insert(pair< double, vector< double > >(basePlaceScore, baseP));
  }
  vector< geometry_msgs::Pose > final_base_loc;
  for (multiset< pair< double, vector< double > > >::iterator it = basePoseWithHits.begin();
       it != basePoseWithHits.end(); ++it)
  {
    geometry_msgs::Pose final_base;
    sd.convertVectorToPose(it->second, final_base);
    final_base_loc.push_back(final_base);
  }

  ROS_INFO_STREAM("Poses that have optimal base location: " << final_base_loc.size());
  if (final_base_loc.size() < BASE_LOC_SIZE_)
    ROS_ERROR_STREAM("The map you have provided is not suitable for base placement. Please provide a valid map.");
  for (int i = 0; i < BASE_LOC_SIZE_; ++i)
  {
    final_base_poses.push_back(final_base_loc[i]);
  }
}

void PlaceBase::showBaseLocationsbyArrow(vector< geometry_msgs::Pose > po)
{
  /* Visualizing base solutions as arrow. Arrows are pointing towards Y direction.(for now)
   Have to move it towards z direction
  */
  ROS_INFO("Showing Base Locations by Arrow:");
  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise< visualization_msgs::MarkerArray >("visualization_marker_array", 1);
  visualization_msgs::MarkerArray markerArr;
  for (int i = 0; i < po.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.3;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.id = i;

    marker.pose.position.x = po[i].position.x;
    marker.pose.position.y = po[i].position.y;
    marker.pose.position.z = po[i].position.z;

    marker.pose.orientation.w = po[i].orientation.w;
    marker.pose.orientation.x = po[i].orientation.x;
    marker.pose.orientation.y = po[i].orientation.y;
    marker.pose.orientation.z = po[i].orientation.z;
    ROS_INFO("Arrow number: [#%i ] ", i + 1);
    // ROS_INFO("received msg [%f] [%f] [%f] [%f] [%f] [%f] [%f]",po[i].position.x,po[i].position.y,po[i].position.z,
    // po[i].orientation.x,po[i].orientation.y, po[i].orientation.z,po[i].orientation.w);
    markerArr.markers.push_back(marker);
  }
  marker_pub.publish(markerArr);
  // ROS_INFO("Marker size: %lu",  markerArr.markers.size());
}

void PlaceBase::showBaseLocationsbyRobotModel(vector< geometry_msgs::Pose > po)
{
  /* Slot for visualizing base solutions as robot model.
  This can be done by loading a robot model from urdf as mentioned in urdf_tutorial
  Then run a vector with multiple robot models but with different position and orientation.
  Need to be implemented in workspace_visualization package.
  Robotmodel in rviz is a child of display class. While taking data from a topic can be implemented by
  messageFilterDisplay class.
  From here we just have to send PoseArray message.
  */
  ROS_INFO("Showing Base Locations by Robot Model:");
  ros::NodeHandle nh;
  /*ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseArray>("robot",1);
  geometry_msgs::PoseArray poArr;
  poArr.header.frame_id="/base_link";
  poArr.header.stamp=ros::Time::now();
  for(int i=0;i<po.size();++i)
  {
    poArr.poses.push_back(po[i]);
    ROS_INFO("received msg [%f] [%f] [%f] [%f] [%f] [%f] [%f]",po[i].position.x,po[i].position.y,po[i].position.z,
  po[i].orientation.x,po[i].orientation.y, po[i].orientation.z,po[i].orientation.w);
    }
  pose_pub.publish(poArr);*/
  ros::Publisher marker_pub = nh.advertise< visualization_msgs::MarkerArray >("visualization_marker_array", 1);
  visualization_msgs::MarkerArray markerArr;
  for (int i = 0; i < po.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.3;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.id = i;

    marker.pose.position.x = po[i].position.x;
    marker.pose.position.y = po[i].position.y;
    marker.pose.position.z = po[i].position.z;

    marker.pose.orientation.w = po[i].orientation.w;
    marker.pose.orientation.x = po[i].orientation.x;
    marker.pose.orientation.y = po[i].orientation.y;
    marker.pose.orientation.z = po[i].orientation.z;
    ROS_INFO("Arrow number: [#%i ] ", i + 1);
    // ROS_INFO("received msg [%f] [%f] [%f] [%f] [%f] [%f] [%f]",po[i].position.x,po[i].position.y,po[i].position.z,
    // po[i].orientation.x,po[i].orientation.y, po[i].orientation.z,po[i].orientation.w);
    markerArr.markers.push_back(marker);
  }
  marker_pub.publish(markerArr);
  // ROS_INFO("Marker size: %lu",  markerArr.markers.size());
}

void PlaceBase::setReachabilityData(std::multimap< std::vector< double >, std::vector< double > > PoseCollection,
                                    std::multimap< std::vector< double >, double > SphereCollection, float resolution)
{
  /* Setting the reachabilty data
  */
  PoseColFilter = PoseCollection;
  SphereCol = SphereCollection;
  res = resolution;
  ROS_INFO("Size of poses dataset: %lu", PoseColFilter.size());
  ROS_INFO("Size of Sphere dataset: %lu", SphereCol.size());
  ROS_INFO("Resolution: %f", res);
}

void PlaceBase::ShowUnionMap(bool show_map)
{
  /* Slot for showing Union map. Can only be visualized after calling the find base function. Otherwise it will show
   * error message
  */
  ROS_INFO("Showing Map:");
  if (sphereColor.size() == 0)
    ROS_INFO("The union map has not been created yet. Please create the Union map by the Find Base button");
  else
  {
    ros::NodeHandle n;
    ros::Publisher workspace_pub = n.advertise< map_creator::WorkSpace >("reachability_map", 1);
    map_creator::WorkSpace ws;
    ws.header.stamp = ros::Time::now();
    ws.header.frame_id = "/base_link";
    ws.resolution = res;

    for (multimap< vector< double >, double >::iterator it = sphereColor.begin(); it != sphereColor.end(); ++it)
    {
      map_creator::WsSphere wss;
      wss.point.x = it->first[0];
      wss.point.y = it->first[1];
      wss.point.z = it->first[2];
      wss.ri = it->second;

      multimap< vector< double >, vector< double > >::iterator it1;
      for (it1 = baseTrnsCol.lower_bound(it->first); it1 != baseTrnsCol.upper_bound(it->first); ++it1)
      {
        geometry_msgs::Pose pp;
        pp.position.x = it1->second[0];
        pp.position.y = it1->second[1];
        pp.position.z = it1->second[2];
        pp.orientation.x = it1->second[3];
        pp.orientation.y = it1->second[4];
        pp.orientation.z = it1->second[5];
        pp.orientation.w = it1->second[6];
        wss.poses.push_back(pp);
      }
      ws.WsSpheres.push_back(wss);
    }

    workspace_pub.publish(ws);
  }
}

void PlaceBase::clearUnionMap(bool clear_map)
{
  /* The function need to be implemented. The slot is already created in widget
  */
  ROS_INFO("Clearing Map:");
}
