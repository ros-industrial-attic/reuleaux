#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ctime>
#include <string>
#include <time.h>
#include <sstream>
#include <map_creator/sphere_discretization.h>
#include <map_creator/kinematics.h>
#include<map_creator/hdf5_dataset.h>

//struct stat st;

bool isFloat(std::string s)
{
  std::istringstream iss(s);
  float dummy;
  iss >> std::noskipws >> dummy;
  return iss && iss.eof();  // Result converted to bool
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "capability_map");
  ros::NodeHandle n;
  time_t startit, finish;
  time(&startit);
  float resolution = 0.08;
  kinematics::Kinematics k;
  std::string ext = ".h5";
  std::string filename =
      str(boost::format("%s_r%d_capability.h5") % k.getRobotName() % resolution);
  if (argc == 2)
  {
    if (!isFloat(argv[1]))
    {
      ROS_ERROR_STREAM("Probably you have just provided only the map filename. Hey!! The first argument is the "
                       "resolution.");
      return 0;
    }
    resolution = atof(argv[1]);
    filename =
        str(boost::format("%s_r%d_capability.h5") % k.getRobotName() % resolution);
  }

  else if (argc == 3)
  {
    std::string name;
    name = argv[2];
    if (!isFloat(argv[1]) && isFloat(argv[2]))
    {
      ROS_ERROR_STREAM("Hey!! The first argument is the resolution and the second argument is the map filename. You "
                       "messed up.");
      return 0;
    }

    else if (name.find(ext) == std::string::npos)
    {
      ROS_ERROR_STREAM("Please provide an extension of .h5 It will make life easy");
      return 0;
    }
    else
    {
      resolution = atof(argv[1]);
      filename = argv[2];
    }
  }
  else if (argc < 2)
  {
    ROS_INFO("You have not provided any argument. So taking default values.");
  }

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    float HI = -1.5, LO = 1.5;
    unsigned char maxDepth = 16;
    unsigned char minDepth = 0;
    // A box of radius 1 is created. It will be the size of the robot+1.5. Then the box is discretized by voxels of
    // specified resolution
    // TODO resolution will be user argument
    // The center of every voxels are stored in a vector

    sphere_discretization::SphereDiscretization sd;
    float r = 1;

    octomap::point3d origin = octomap::point3d(0, 0, 0);  // This point will be the base of the robot
    octomap::OcTree *tree = sd.generateBoxTree(origin, r, resolution);
    std::vector< octomap::point3d > newData;
    ROS_INFO("Creating the box and discretizing with resolution: %f", resolution);
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(maxDepth), end = tree->end_leafs(); it != end; ++it)
    {
      newData.push_back(it.getCoordinate());
    }
    ROS_INFO("Total no of spheres now: %lu", newData.size());
    ROS_INFO("Please hold ON. Spheres are discretized and all of the poses are checked for Ik solutions. May take some "
             "time");

    float radius = resolution;

    std::multimap< std::vector< double >, std::vector< double > > PoseCol;
    {
      std::vector< geometry_msgs::Pose > pose;
      std::vector< double > sphere_coord;
      std::vector< double > point_on_sphere;
      for (int i = 0; i < newData.size(); i++)
      {
        sd.convertPointToVector(newData[i], sphere_coord);

        sd.make_sphere_poses(newData[i], radius, pose);
        for (int j = 0; j < pose.size(); j++)
        {
          sd.convertPoseToVector(pose[j], point_on_sphere);
          PoseCol.insert(std::make_pair(point_on_sphere, sphere_coord));
        }
      }
    }

    // Every pose is checked for IK solutions. The reachable poses and the their corresponsing joint solutions are
    // stored. Only the First joint solution is stored. We may need this solutions in the future. Otherwise we can show
    // the robot dancing with the joint solutions in a parallel thread
    // TODO Support for more than 6DOF robots needs to be implemented.

    kinematics::Kinematics k;
    std::multimap< std::vector< double >, std::vector< double > > PoseColFilter;
    std::multimap< std::vector< double >, std::vector< double > > PoseCol2;
    std::vector< std::vector< double > > ikSolutions;
    for (std::multimap< std::vector< double >, std::vector< double > >::iterator it = PoseCol.begin(); it != PoseCol.end(); ++it)
    {
      std::vector< double > joints;
      joints.resize(6);
      PoseCol2.insert(std::pair< std::vector< double >, std::vector< double > >(it->second, it->first));
      int solns;
      if (k.isIKSuccess(it->first, joints, solns))
      {
        PoseColFilter.insert(std::pair< std::vector< double >, std::vector< double > >(it->second, it->first));
        ikSolutions.push_back(joints);
      }
    }

    ROS_INFO("Total number of poses: %lu", PoseCol.size());
    ROS_INFO("Total number of reachable poses: %lu", PoseColFilter.size());

    // The centers of reachable spheres are stored in a map. This data will be utilized in visualizing the spheres in
    // the visualizer.
    // TODO there are several maps are implemented. We can get rid of few maps and run large loops. The complexity of
    // accessing map is Olog(n)
    std::vector< std::vector< double > > capability_data;
    std::map< std::vector< double >, double > sphereColor;
    std::vector< std::vector< double > > poseReach;
    for (std::multimap< std::vector< double >, std::vector< double > >::iterator it = PoseColFilter.begin(); it != PoseColFilter.end();
         ++it)
    {
      // Reachability Index D=R/N*100;

      float d = float(PoseColFilter.count(it->first)) / (PoseCol.size() / newData.size()) * 100;
      sphereColor.insert(std::pair< std::vector< double >, double >(it->first, double(d)));
      poseReach.push_back(it->second);
    }

    ROS_INFO("No of spheres reachable: %lu", sphereColor.size());

    // Starting capability map

    ROS_INFO("All the outer spheres are checked for optimal pose and optimal openning angles for cone representation. "
             "May take some time.");
    int i = 0;
    for (std::map< std::vector< double >, double >::iterator it = sphereColor.begin(); it != sphereColor.end();
         ++it)  // for all the spheres in workspace
    {
      i += 1;
      ROS_INFO ("Processing sphere: %d", i);

      if (it->second <= 20)  // All the spheres that have reachability less or equal to 20
      {
        octomap::point3d sphereCenter;
        sd.convertVectorToPoint(it->first, sphereCenter);  // center of sphere

        std::vector< geometry_msgs::Pose > reachPoseofSphere;
        std::vector< octomap::point3d > reachPoints;

        std::multimap< std::vector< double >, std::vector< double > >::iterator it1;  // Looking for poses of those spheres
        for (it1 = PoseColFilter.lower_bound(it->first); it1 != PoseColFilter.upper_bound(it->first); ++it1)
        {
          geometry_msgs::Pose pp;
          sd.convertVectorToPose(it1->second, pp);  // found poses for spheres

          octomap::point3d posPoint;
          sd.poseToPoint(pp, posPoint);  // only positions are taken from thoses poses

          reachPoseofSphere.push_back(pp);  // poses in a vector
          reachPoints.push_back(posPoint);  // filtered positions in a vector
        }
        geometry_msgs::Pose optiPose;

        // finding optimal pose of the sphere
        geometry_msgs::Pose optimal_pose_pca;
        sd.findOptimalPosebyPCA(reachPoseofSphere, optimal_pose_pca);

        optiPose.position.x = sphereCenter.x();
        optiPose.position.y = sphereCenter.y();
        optiPose.position.z = sphereCenter.z();
        optiPose.orientation.x = optimal_pose_pca.orientation.x;
        optiPose.orientation.y = optimal_pose_pca.orientation.y;
        optiPose.orientation.z = optimal_pose_pca.orientation.z;
        optiPose.orientation.w = optimal_pose_pca.orientation.w;

        double SFE = 0.0;
        std::map< double, double > angleSFE;
        for (double angle = 2; angle <= 10.0; angle += 0.5)
        {
          pcl::PointCloud< pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZ >);
          sd.createConeCloud(optiPose, angle, 0.5, cloud);
          //cout << "cloud size: " << cloud->size() << endl;
          //ROS_INFO_STREAM("cloud size: " << cloud->size());
          double r_poses = 0.0;  // Pose that are reachable but not in cone
          for (int j = 0; j < reachPoints.size(); j++)
          {
            if (!sd.isPointInCloud(cloud, reachPoints[j]))
              r_poses += 1;
          }

          double R_poses = reachPoints.size();  // Total number of filtered pose in that sphere

          std::multimap< std::vector< double >, std::vector< double > >::iterator it2;
          std::vector< octomap::point3d > reachPointsSphere;
          for (it2 = PoseCol2.lower_bound(it->first); it2 != PoseCol2.upper_bound(it->first); ++it2)
          {
            geometry_msgs::Pose pp;
            octomap::point3d posPoint;
            sd.convertVectorToPose(it2->second, pp);
            sd.poseToPoint(pp, posPoint);
            reachPointsSphere.push_back(posPoint);
          }
          double v_poses = 0.0;  // poses that are in the cone but not reachable
          for (int k = 0; k < reachPointsSphere.size(); k++)
          {
            if (sd.isPointInCloud(cloud, reachPointsSphere[k]))
              if (std::count(reachPoints.begin(), reachPoints.end(), reachPointsSphere[k]) == 0)
                v_poses += 1;
          }
          SFE = (r_poses + v_poses) / R_poses;
          angleSFE.insert(std::pair< double, double >(SFE, angle));
        }
        std::vector< double > capability;
        capability.push_back(1.0);  // Enum for cone
        capability.push_back(it->second);  // Reachability index
        capability.push_back(angleSFE.begin()->second);  // Optimal cone angle
        capability.push_back(optiPose.position.x);  // Position x,y,z
        capability.push_back(optiPose.position.y);
        capability.push_back(optiPose.position.z);
        capability.push_back(optiPose.orientation.x);  // Orientation x,y,z,w
        capability.push_back(optiPose.orientation.y);
        capability.push_back(optiPose.orientation.z);
        capability.push_back(optiPose.orientation.w);
        capability_data.push_back(capability);
      }

      else
      {
        std::vector< double > capability_sp;
        capability_sp.push_back(2.0);
        capability_sp.push_back(it->second);
        capability_sp.push_back(0.0);
        capability_sp.push_back(it->first[0]);
        capability_sp.push_back(it->first[1]);
        capability_sp.push_back(it->first[2]);
        capability_sp.push_back(0.0);
        capability_sp.push_back(0.0);
        capability_sp.push_back(0.0);
        capability_sp.push_back(1.0);
        capability_data.push_back(capability_sp);
      }
    }
    ROS_INFO("Capability map is created, saving data to database.");

    // Saving to database
    hdf5_dataset::Hdf5Dataset h5(filename);
    h5.saveCapMapsToDataset(capability_data, resolution);


    time(&finish);
    double dif = difftime(finish, startit);
    ROS_INFO("Elasped time is %.2lf seconds.", dif);
    ROS_INFO("Completed");
    ros::spinOnce();
    // sleep(20000);
    return 1;
    loop_rate.sleep();
    count;
  }

  return 0;
}
