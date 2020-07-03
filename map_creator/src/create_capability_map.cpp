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
#include <map_creator/hdf5_dataset.h>
#include <boost/format.hpp>

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
    unsigned char max_depth = 16;
    unsigned char minDepth = 0;
    // A box of radius 1 is created. It will be the size of the robot+1.5. Then the box is discretized by voxels of
    // specified resolution
    // TODO resolution will be user argument
    // The center of every voxels are stored in a vector

    sphere_discretization::SphereDiscretization sd;
    float r = 1;

    octomap::point3d origin = octomap::point3d(0, 0, 0);  // This point will be the base of the robot
    octomap::OcTree *tree = sd.generateBoxTree(origin, r, resolution);
    std::vector< octomap::point3d > new_data;
    ROS_INFO("Creating the box and discretizing with resolution: %f", resolution);
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
      new_data.push_back(it.getCoordinate());
    }
    ROS_INFO("Total no of spheres now: %lu", new_data.size());
    ROS_INFO("Please hold ON. Spheres are discretized and all of the poses are checked for Ik solutions. May take some "
             "time");

    float radius = resolution;

    std::multimap< std::vector< double >, std::vector< double > > pose_col;
    {
      std::vector< geometry_msgs::Pose > pose;
      std::vector< double > sphere_coord;
      std::vector< double > point_on_sphere;
      for (int i = 0; i < new_data.size(); i++)
      {
        sd.convertPointToVector(new_data[i], sphere_coord);

        sd.make_sphere_poses(new_data[i], radius, pose);
        for (int j = 0; j < pose.size(); j++)
        {
          sd.convertPoseToVector(pose[j], point_on_sphere);
          pose_col.insert(std::make_pair(point_on_sphere, sphere_coord));
        }
      }
    }

    // Every pose is checked for IK solutions. The reachable poses and the their corresponsing joint solutions are
    // stored. Only the First joint solution is stored. We may need this solutions in the future. Otherwise we can show
    // the robot dancing with the joint solutions in a parallel thread
    // TODO Support for more than 6DOF robots needs to be implemented.

    kinematics::Kinematics k;
    std::multimap< std::vector< double >, std::vector< double > > pose_col_filter;
    std::multimap< std::vector< double >, std::vector< double > > pose_col2;
    std::vector< std::vector< double > > ik_solutions;
    for (std::multimap< std::vector< double >, std::vector< double > >::iterator it = pose_col.begin(); it != pose_col.end(); ++it)
    {
      std::vector< double > joints;
      joints.resize(6);
      pose_col2.insert(std::pair< std::vector< double >, std::vector< double > >(it->second, it->first));
      int solns;
      if (k.isIKSuccess(it->first, joints, solns))
      {
        pose_col_filter.insert(std::pair< std::vector< double >, std::vector< double > >(it->second, it->first));
        ik_solutions.push_back(joints);
      }
    }

    ROS_INFO("Total number of poses: %lu", pose_col.size());
    ROS_INFO("Total number of reachable poses: %lu", pose_col_filter.size());

    // The centers of reachable spheres are stored in a map. This data will be utilized in visualizing the spheres in
    // the visualizer.
    // TODO there are several maps are implemented. We can get rid of few maps and run large loops. The complexity of
    // accessing map is Olog(n)
    std::vector< std::vector< double > > capability_data;
    std::map< std::vector< double >, double > sphere_color;
    std::vector< std::vector< double > > poseReach;
    for (std::multimap< std::vector< double >, std::vector< double > >::iterator it = pose_col_filter.begin(); it != pose_col_filter.end();
         ++it)
    {
      // Reachability Index D=R/N*100;

      float d = float(pose_col_filter.count(it->first)) / (pose_col.size() / new_data.size()) * 100;
      sphere_color.insert(std::pair< std::vector< double >, double >(it->first, double(d)));
      poseReach.push_back(it->second);
    }

    ROS_INFO("No of spheres reachable: %lu", sphere_color.size());

    // Starting capability map

    ROS_INFO("All the outer spheres are checked for optimal pose and optimal openning angles for cone representation. "
             "May take some time.");
    int i = 0;
    for (std::map< std::vector< double >, double >::iterator it = sphere_color.begin(); it != sphere_color.end();
         ++it)  // for all the spheres in workspace
    {
      i += 1;
      ROS_INFO ("Processing sphere: %d", i);

      if (it->second <= 20)  // All the spheres that have reachability less or equal to 20
      {
        octomap::point3d sphere_center;
        sd.convertVectorToPoint(it->first, sphere_center);  // center of sphere

        std::vector< geometry_msgs::Pose > reach_pose_of_sphere;
        std::vector< octomap::point3d > reach_points;

        std::multimap< std::vector< double >, std::vector< double > >::iterator it1;  // Looking for poses of those spheres
        for (it1 = pose_col_filter.lower_bound(it->first); it1 != pose_col_filter.upper_bound(it->first); ++it1)
        {
          geometry_msgs::Pose pp;
          sd.convertVectorToPose(it1->second, pp);  // found poses for spheres

          octomap::point3d pose_point;
          sd.poseToPoint(pp, pose_point);  // only positions are taken from thoses poses

          reach_pose_of_sphere.push_back(pp);  // poses in a vector
          reach_points.push_back(pose_point);  // filtered positions in a vector
        }
        geometry_msgs::Pose opti_pose;

        // finding optimal pose of the sphere
        geometry_msgs::Pose optimal_pose_pca;
        sd.findOptimalPosebyPCA(reach_pose_of_sphere, optimal_pose_pca);

        opti_pose.position.x = sphere_center.x();
        opti_pose.position.y = sphere_center.y();
        opti_pose.position.z = sphere_center.z();
        opti_pose.orientation.x = optimal_pose_pca.orientation.x;
        opti_pose.orientation.y = optimal_pose_pca.orientation.y;
        opti_pose.orientation.z = optimal_pose_pca.orientation.z;
        opti_pose.orientation.w = optimal_pose_pca.orientation.w;

        double sfe = 0.0;
        std::map< double, double > angle_sfe;
        for (double angle = 2; angle <= 10.0; angle += 0.5)
        {
          pcl::PointCloud< pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZ >);
          sd.createConeCloud(opti_pose, angle, 0.5, cloud);
          //cout << "cloud size: " << cloud->size() << endl;
          //ROS_INFO_STREAM("cloud size: " << cloud->size());
          double r_poses = 0.0;  // Pose that are reachable but not in cone
          for (int j = 0; j < reach_points.size(); j++)
          {
            if (!sd.isPointInCloud(cloud, reach_points[j]))
              r_poses += 1;
          }

          double R_poses = reach_points.size();  // Total number of filtered pose in that sphere

          std::multimap< std::vector< double >, std::vector< double > >::iterator it2;
          std::vector< octomap::point3d > reach_points_sphere;
          for (it2 = pose_col2.lower_bound(it->first); it2 != pose_col2.upper_bound(it->first); ++it2)
          {
            geometry_msgs::Pose pp;
            octomap::point3d pose_point;
            sd.convertVectorToPose(it2->second, pp);
            sd.poseToPoint(pp, pose_point);
            reach_points_sphere.push_back(pose_point);
          }
          double v_poses = 0.0;  // poses that are in the cone but not reachable
          for (int k = 0; k < reach_points_sphere.size(); k++)
          {
            if (sd.isPointInCloud(cloud, reach_points_sphere[k]))
              if (std::count(reach_points.begin(), reach_points.end(), reach_points_sphere[k]) == 0)
                v_poses += 1;
          }
          sfe = (r_poses + v_poses) / R_poses;
          angle_sfe.insert(std::pair< double, double >(sfe, angle));
        }
        std::vector< double > capability;
        capability.push_back(1.0);  // Enum for cone
        capability.push_back(it->second);  // Reachability index
        capability.push_back(angle_sfe.begin()->second);  // Optimal cone angle
        capability.push_back(opti_pose.position.x);  // Position x,y,z
        capability.push_back(opti_pose.position.y);
        capability.push_back(opti_pose.position.z);
        capability.push_back(opti_pose.orientation.x);  // Orientation x,y,z,w
        capability.push_back(opti_pose.orientation.y);
        capability.push_back(opti_pose.orientation.z);
        capability.push_back(opti_pose.orientation.w);
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
