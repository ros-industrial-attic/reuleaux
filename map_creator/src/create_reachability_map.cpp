// The spheres and poses are fused in a single dataset, instead of two datasets for sphere and poses

#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <map_creator/sphere_discretization.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseArray.h"
#include <map>
#include <map_creator/kinematics.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ctime>
#include "H5Cpp.h"
#include <hdf5.h>
#include <string>
#include <time.h>
#include <sstream>

// TODO get rid of all the headers that are not needed
using namespace H5;
using namespace octomap;
using namespace std;
using namespace octomath;
using namespace sphere_discretization;
using namespace kinematics;

struct stat st;

typedef vector<std::pair< vector< double >, const vector< double >* > > MultiVector;
typedef multimap< const vector< double >*, const vector< double >* > MultiMap;

bool isFloat(string s)
{
  istringstream iss(s);
  float dummy;
  iss >> noskipws >> dummy;
  return iss && iss.eof();  // Result converted to bool
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "workspace");
  ros::NodeHandle n;
  ros::Time startit = ros::Time::now();

  float resolution = 0.06;
  Kinematics k;
  string ext = ".h5";
  string filename =
      string(k.getRobotName()) + "_" + "r" + str(boost::format("%d") % resolution) + "_" + "reachability" + "." + "h5";
  if (argc == 2)
  {
    if (!isFloat(argv[1]))
    {
      ROS_ERROR_STREAM("Probably you have just provided only the map filename. Hey!! The first argument is the "
                       "resolution.");
      return 0;
    }
    resolution = atof(argv[1]);
    filename = string(k.getRobotName()) + "_" + "r" + str(boost::format("%d") % resolution) + "_" + "reachability" +
               "." + "h5";
  }

  else if (argc == 3)
  {
    string name;
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
  // ros::Publisher workspace_pub = n.advertise<map_creator::WorkSpace>("workspace", 10);
  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok())
  {
    unsigned char maxDepth = 16;
    unsigned char minDepth = 0;

    // A box of radius 1 is created. It will be the size of the robot+1.5. Then the box is discretized by voxels of
    // specified resolution
    // TODO resolution will be user argument
    // The center of every voxels are stored in a vector

    SphereDiscretization sd;
    float r = 1;
    point3d origin = point3d(0, 0, 0);  // This point will be the base of the robot
    OcTree *tree = sd.generateBoxTree(origin, r, resolution);
    std::vector< point3d > newData;
    ROS_INFO("Creating the box and discretizing with resolution: %f", resolution);
    for (OcTree::leaf_iterator it = tree->begin_leafs(maxDepth), end = tree->end_leafs(); it != end; ++it)
    {
      newData.push_back(it.getCoordinate());
    }

    ROS_INFO("Total no of spheres now: %lu", newData.size());
    ROS_INFO("Please hold ON. Spheres are discretized and all of the poses are checked for Ik solutions. May take some "
             "time");

    // A sphere is created in every voxel. The sphere may be created by default or other techniques.
    // TODO Other techniques need to be modified. the user can specifiy which technique they want to use
    // TODO The sphere discretization parameter and rotation of every poses will be taken as argument. If the final
    // joints can rotate (0, 2pi) we dont need to rotate the poses.
    // Every discretized points on spheres are converted to pose and all the poses are saved in a multimap with their
    // corresponding sphere centers
    // If the resolution is 0.01 the programs not responds

    float radius = resolution;
    vector< vector< double > > SphereCoord;
    SphereCoord.resize( newData.size() );

    MultiVector PoseCol;
    PoseCol.reserve( newData.size() * 50);

    for (int i = 0; i < newData.size(); i++)
    {
      vector< geometry_msgs::Pose > pose;
      sd.convertPointToVector(newData[i], SphereCoord[i]);

      pose = sd.make_sphere_poses(newData[i], radius);
      for (int j = 0; j < pose.size(); j++)
      {
        vector< double > point_on_sphere;
        sd.convertPoseToVector(pose[j], point_on_sphere);

        PoseCol.push_back( std::make_pair(point_on_sphere, &SphereCoord[i]));
      }
    }

    // Every pose is checked for IK solutions. The reachable poses and the their corresponsing joint solutions are
    // stored. Only the First joint solution is stored. We may need this solutions in the future. Otherwise we can show
    // the robot dancing with the joint solutions in a parallel thread
    // TODO Support for more than 6DOF robots needs to be implemented.

    // Kinematics k;
    MultiMap PoseColFilter;
    vector< vector< double > > ikSolutions;
    ikSolutions.reserve( PoseCol.size() );

    for (MultiVector::iterator it = PoseCol.begin(); it != PoseCol.end(); ++it)
    {
      std::vector< double > joints;
      joints.resize(6);
      int solns;
      if (k.isIKSuccess(it->first, joints, solns))
      {
        PoseColFilter.insert( std::make_pair( it->second, &(it->first)));
        ikSolutions.push_back(joints);
        // cout<<it->first[0]<<" "<<it->first[1]<<" "<<it->first[2]<<" "<<it->first[3]<<" "<<it->first[4]<<"
        // "<<it->first[5]<<" "<<it->first[6]<<endl;
      }
    }

    ROS_INFO("Total number of poses: %lu", PoseCol.size());
    ROS_INFO("Total number of reachable poses: %lu", PoseColFilter.size());

    // The centers of reachable spheres are stored in a map. This data will be utilized in visualizing the spheres in
    // the visualizer.
    // TODO there are several maps are implemented. We can get rid of few maps and run large loops. The complexity of
    // accessing map is Olog(n)

    map< const vector< double >*, double > sphereColor;
    vector< vector< double > > poseReach;

    for (MultiMap::iterator it = PoseColFilter.begin(); it != PoseColFilter.end(); ++it)
    {
      const vector<double>* sphere_coord    = it->first;
      const vector<double>* point_on_sphere = it->second;

      // Reachability Index D=R/N*100;
      float d = float(PoseColFilter.count(sphere_coord)) / (PoseCol.size() / newData.size()) * 100;
      sphereColor.insert( std::make_pair(it->first, double(d)));

      // poseReach.push_back(it->second);
      vector< double > poseAndSphere;
      poseAndSphere.reserve( sphere_coord->size() + point_on_sphere->size());

      for (int i = 0; i < sphere_coord->size(); i++)
      {
        poseAndSphere.push_back((*it->first)[i]);
      }
      for (int j = 0; j < point_on_sphere->size(); j++)
      {
        poseAndSphere.push_back((*it->second)[j]);
      }

      poseReach.push_back(poseAndSphere);
    }

    ROS_INFO("No of spheres reachable: %lu", sphereColor.size());

    // Creating maps now

    string path(ros::package::getPath("map_creator") + "/maps/");
    if (stat(path.c_str(), &st) != 0)
      ROS_INFO("Path does not exist. Creating folder for maps");
    const int dir_err = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (1 == dir_err)
    {
      ROS_INFO("Error creating directory");
      exit(1);
    }

    // Time is taken for saving the filename of the map
    // TODO the filename will be an argument
    // TODO need to include the sphere discretization and pose rotation parameters in the name
    // If the user does not provide the desired name, this will be default name of the map created in the "maps" folder.
    // robot_time_date_resoltion_spdis_porot.h5

    time_t currentTime;
    struct tm *localTime;
    time(&currentTime);
    localTime = localtime(&currentTime);

    int Day = localTime->tm_mday;
    int Month = localTime->tm_mon + 1;
    int Year = localTime->tm_year + 1900;
    int Hour = localTime->tm_hour;
    int Min = localTime->tm_min;
    int Sec = localTime->tm_sec;

    // Creating all the file and group ids and the default file name

    // string filename;
    //    filename=string(k.getRobotName())+"_"+boost::lexical_cast<std::string>(Hour)+":"+boost::lexical_cast<std::string>(Min)+"_"+boost::lexical_cast<std::string>(Month)+":"+boost::lexical_cast<std::string>(Day)+":"+boost::lexical_cast<std::string>(Year)+"_"+"r"+str(
    //    boost::format("%d") % resolution)+"_"+"sd"+"_"+"rot"+"_"+"reachability"+"."+"h5";

    // The filename is shortened for now for testing purpose.
    // filename=string(k.getRobotName())+"_"+"r"+str( boost::format("%d") % resolution)+"_"+"reachability"+"."+"h5";

    const char *filepath = path.c_str();
    const char *name = filename.c_str();
    char fullpath[100];
    strcpy(fullpath, filepath);
    strcat(fullpath, name);
    ROS_INFO("Saving map %s", filename.c_str());
    hid_t file, group_poses, group_spheres;
    file = H5Fcreate(fullpath, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);

    group_poses = H5Gcreate(file, "/Poses", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    group_spheres = H5Gcreate(file, "/Spheres", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    // Creating Poses dataset: This process is little complicated. As the resolution decreses, e.g 0.03, the number of
    // poses rises upto 7 digits, so the matrix of double value is not going to support this. We are creating a chunked
    // infinite dataset which will select memory hyperslab and write data to it from buffer. We are using 2 hyperslabs.

    ROS_INFO("Saving poses in reachability map");

    const hsize_t ndims = 2;
    const hsize_t ncols = 10;

    int posSize = PoseColFilter.size();
    int chunk_size;
    int PY = 10;
    if (posSize % 2)
    {
      chunk_size = (posSize / 2) + 1;
    }
    else
    {
      chunk_size = (posSize / 2);
    }

    // Create Dataspace
    hsize_t dims[ndims] = {0, ncols};  // Starting with an empty buffer
    hsize_t max_dims[ndims] = {H5S_UNLIMITED, ncols};  // Creating dataspace
    hid_t file_space = H5Screate_simple(ndims, dims, max_dims);

    // Create Dataset Property list
    hid_t plist = H5Pcreate(H5P_DATASET_CREATE);
    H5Pset_layout(plist, H5D_CHUNKED);
    hsize_t chunk_dims[ndims] = {chunk_size, ncols};
    H5Pset_chunk(plist, ndims, chunk_dims);

    // Create the datset
    hid_t dset = H5Dcreate(group_poses, "poses_dataset", H5T_NATIVE_FLOAT, file_space, H5P_DEFAULT, plist, H5P_DEFAULT);

    // Closing resources
    H5Pclose(plist);
    H5Sclose(file_space);

    // Creating the first buffer
    hsize_t nlines = chunk_size;
    float *buffer = new float[nlines * ncols];
    float **dset1_data = new float *[nlines];
    for (hsize_t i = 0; i < nlines; ++i)
    {
      dset1_data[i] = &buffer[i * ncols];
    }

    // Data for the first chunk
    for (int i = 0; i < chunk_size; i++)
    {
      for (int j = 0; j < PY; j++)
      {
        dset1_data[i][j] = poseReach[i][j];
      }
    }

    // Memory dataspace indicating size of the buffer
    dims[0] = chunk_size;
    dims[1] = ncols;
    hid_t mem_space = H5Screate_simple(ndims, dims, NULL);

    // Extending dataset
    dims[0] = chunk_size;
    dims[1] = ncols;
    H5Dset_extent(dset, dims);

    // Selecting hyperslab on the dataset
    file_space = H5Dget_space(dset);
    hsize_t start[2] = {0, 0};
    hsize_t count[2] = {chunk_size, ncols};
    H5Sselect_hyperslab(file_space, H5S_SELECT_SET, start, NULL, count, NULL);

    // Writing buffer to the dataset
    H5Dwrite(dset, H5T_NATIVE_FLOAT, mem_space, file_space, H5P_DEFAULT, buffer);

    // Closing file dataspace
    H5Sclose(file_space);

    // Data for the Second chunk
    for (int i = chunk_size; i < posSize; i++)
    {
      for (int j = 0; j < PY; j++)
      {
        dset1_data[i - chunk_size][j] = poseReach[i][j];
      }
    }

    // Resizing new memory dataspace indicating new size of the buffer
    dims[0] = posSize - chunk_size;
    dims[1] = ncols;
    H5Sset_extent_simple(mem_space, ndims, dims, NULL);

    // Extend dataset
    dims[0] = posSize;
    dims[1] = ncols;
    H5Dset_extent(dset, dims);

    // Selecting hyperslab
    file_space = H5Dget_space(dset);
    start[0] = chunk_size;
    start[1] = 0;
    count[0] = posSize - chunk_size;
    count[1] = ncols;
    H5Sselect_hyperslab(file_space, H5S_SELECT_SET, start, NULL, count, NULL);

    // Writing buffer to dataset
    H5Dwrite(dset, H5T_NATIVE_FLOAT, mem_space, file_space, H5P_DEFAULT, buffer);

    // Closing all the resources
    delete[] dset1_data;
    delete[] buffer;
    H5Sclose(file_space);
    H5Sclose(mem_space);
    H5Dclose(dset);
    H5Gclose(group_poses);

    // This part map not be needed for future work, as the sphere data is already stored in the poses dataset
    // Creating Sphere dataset
    ROS_INFO("Saving spheres in Reachability map");
    hid_t reachability_sphere_dataset, reachability_sphere_dataspace;
    const int SX = sphereColor.size();
    const int SY = 4;

    hsize_t dims2[2];  // dataset dimensions
    dims2[0] = SX;
    dims2[1] = SY;
    double dset2_data[SX][SY];

    for (map< const vector< double >*, double >::iterator it = sphereColor.begin(); it != sphereColor.end(); ++it)
    {
      for (int j = 0; j < SY - 1; j++)
      {
        dset2_data[distance(sphereColor.begin(), it)][j] = (*it->first)[j];
      }
      for (int j = 3; j < SY; j++)
      {
        dset2_data[distance(sphereColor.begin(), it)][j] = it->second;
      }
    }
    reachability_sphere_dataspace = H5Screate_simple(2, dims2, NULL);
    reachability_sphere_dataset = H5Dcreate2(group_spheres, "sphere_dataset", H5T_NATIVE_DOUBLE,
                                             reachability_sphere_dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    H5Dwrite(reachability_sphere_dataset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, dset2_data);

    // Creating attribute

    hid_t attr_id;
    hsize_t attr_dims;
    float attr_data[1];
    attr_data[0] = resolution;
    attr_dims = 1;
    reachability_sphere_dataspace = H5Screate_simple(1, &attr_dims, NULL);
    attr_id = H5Acreate2(reachability_sphere_dataset, "Resolution", H5T_NATIVE_FLOAT, reachability_sphere_dataspace,
                         H5P_DEFAULT, H5P_DEFAULT);
    H5Awrite(attr_id, H5T_NATIVE_FLOAT, attr_data);
    H5Aclose(attr_id);

    // Closing all

    H5Sclose(reachability_sphere_dataspace);
    H5Dclose(reachability_sphere_dataset);
    H5Gclose(group_spheres);

    H5Fclose(file);

	double dif = ros::Duration( ros::Time::now() - startit).toSec();
    ROS_INFO("Elasped time is %.2lf seconds.", dif);
    ROS_INFO("Completed");
    ros::spinOnce();
    // sleep(10000);
    return 1;
    loop_rate.sleep();
    count;
  }
  return 0;
}
