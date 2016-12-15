#ifndef HDF5_DATASET_H
#define HDF5_DATASET_H
#include "H5Cpp.h"
#include <hdf5.h>
#include <iostream>
#include <ros/ros.h>

#include <sys/stat.h>
#include<unistd.h>

typedef std::multimap< const std::vector< double >*, const std::vector< double >* > MultiMap;
typedef std::map< const std::vector< double >*, double > Map;
typedef std::multimap< std::vector< double >, std::vector< double > > multiMap;
typedef std::map< std::vector< double >, double > map_D;
typedef std::vector<std::vector<double> > Vector;
struct stat st;



namespace hdf5_dataset
{
class Hdf5Dataset
{
public:
   Hdf5Dataset(std::string path, std::string filename); //Constructor for accessing .h5 file with path and filename
   Hdf5Dataset(std::string fullpath);// Constructor for accessing .h5 file with only fullpath

  //~Hdf5Dataset();
  bool open(); //Opening the file, groups and database
  bool open_cap(); //Opening the file, groups and database for capability map
  void close_cap();//Closing resources for cap map
  void close(); //Closing all the resources

  bool saveCapMapsToDataset(Vector &capabilityData, float &resolution); //Saves vector of capability data to multimap
  bool saveReachMapsToDataset( MultiMap& Poses, Map& Spheres, float resolution); //Saves Mutimap and Map to database and closes

  bool loadCapMapFromDataset(Vector &capabilityData, float &resolution);//loads capability map and resolution data to
  bool loadMapsFromDataset(MultiMap& Poses, Map& Spheres); //Creates exact same Poses MultiMap that was stored with address variation
  bool loadMapsFromDataset(MultiMap& Poses, Map& Spheres, float &resolution); //with resolution
  bool loadMapsFromDataset(multiMap& Poses, map_D &Spheres); //Loads the pose and sphere
  bool loadMapsFromDataset(multiMap& Poses, map_D &Spheres, float &resolution); //with resolution
  bool h5ToResolution(float &resolution);//Accesses the resolution of the map

private:

  bool h5ToVectorCap(Vector &capabilityData); //Accesses the capability data
  bool h5ToMultiMapPosesAndSpheres(MultiMap  &PoseCol, Map &SphereCol); //loads the whole data with same address structure as stored in .h5

  bool h5ToMultiMapPoses(multiMap &PoseCol, map_D &SphereCol); //accesses the poses and spheres data in the poses dataset
  bool h5ToMultiMapPoses(multiMap &PoseCol); //Accessess only the data from poses dataset regardless of address
  bool h5ToMultiMapSpheres(map_D& SphereCol); //Accessess only the data from spheres dataset regardless of address

  bool checkPath(std::string path); //Checking if path exists
  bool checkFileName(std::string filename); //Checking if filename is a .h5 or not
  void createPath(std::string path); //Creating the path



  std::string path_;
  std::string filename_;
  hid_t file_, group_poses_, group_spheres_, group_capability_;
  hid_t poses_dataset_, sphere_dataset_, capability_dataset_;
  hid_t attr_;
  float res_;
 
};

}  // namespace hdf5_dataset
#endif  // HDF5_DATASET_H
