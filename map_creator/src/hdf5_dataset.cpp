#include <map_creator/hdf5_dataset.h>

#define RANK_OUT 2

namespace hdf5_dataset
{
// Hdf5Dataset::Hdf5Dataset(){}

Hdf5Dataset::Hdf5Dataset(std::string fullpath)
{
  std::stringstream fp(fullpath);
  std::string segment;
  std::vector<std::string> seglist;
  char the_path[256];

  while(std::getline(fp, segment, '/'))
  {
     seglist.push_back(segment);
  }

  std::ostringstream oss_file;
  oss_file<< seglist.back();
  this->filename_ = oss_file.str();


  seglist.pop_back();
  std::ostringstream oss_path;
  if (!seglist.empty())
  {
    std::copy(seglist.begin(), seglist.end()-1,
    std::ostream_iterator<std::string>(oss_path, "/"));
    oss_path << seglist.back();
    oss_path<<"/";
  }
  else
  {
    getcwd(the_path, 255);
    strcat(the_path, "/");
    oss_path << the_path;
  }

 this->path_ = oss_path.str();
 checkPath(this->path_);
 checkFileName(this->filename_);
}

Hdf5Dataset::Hdf5Dataset(std::string path, std::string filename)
{
  this->path_ = path;
  this->filename_ = filename;
  checkPath(this->path_);
  checkFileName(this->filename_);
}

bool Hdf5Dataset::open()
{
  const char *filepath = this->path_.c_str();
  const char *name = this->filename_.c_str();
  char fullpath[100];
  strcpy(fullpath, filepath);
  strcat(fullpath, name);
  ROS_INFO("Opening map %s", this->filename_.c_str());
  this->file_ = H5Fopen(fullpath,  H5F_ACC_RDONLY, H5P_DEFAULT);

  this->group_poses_ = H5Gopen(this->file_, "/Poses", H5P_DEFAULT);
  this->poses_dataset_ = H5Dopen(this->group_poses_, "poses_dataset", H5P_DEFAULT);

  this->group_spheres_ = H5Gopen(this->file_, "/Spheres", H5P_DEFAULT);
  this->sphere_dataset_ = H5Dopen(this->group_spheres_, "sphere_dataset", H5P_DEFAULT);

  this->attr_ = H5Aopen(this->sphere_dataset_, "Resolution", H5P_DEFAULT);
  herr_t ret = H5Aread(this->attr_, H5T_NATIVE_FLOAT, &this->res_);

}

bool Hdf5Dataset::open_cap()
{
  const char *filepath = this->path_.c_str();
  const char *name = this->filename_.c_str();
  char fullpath[100];
  strcpy(fullpath, filepath);
  strcat(fullpath, name);
  ROS_INFO("Opening Capability map %s", this->filename_.c_str());
  this->file_ = H5Fopen(fullpath,  H5F_ACC_RDONLY, H5P_DEFAULT);

  this->group_capability_ = H5Gopen(this->file_, "/Capability", H5P_DEFAULT);
  this->capability_dataset_= H5Dopen(this->group_capability_, "capability_dataset", H5P_DEFAULT);

  this->attr_ = H5Aopen(this->capability_dataset_, "Resolution", H5P_DEFAULT);
  herr_t ret = H5Aread(this->attr_, H5T_NATIVE_FLOAT, &this->res_);

}

void Hdf5Dataset::close()
{
  H5Aclose(this->attr_);
  H5Dclose(this->poses_dataset_);
  H5Gclose(this->group_poses_);
  H5Dclose(this->sphere_dataset_);
  H5Gclose(this->group_spheres_);
  H5Fclose(this->file_);
}

void Hdf5Dataset::close_cap()
{
  H5Aclose(this->attr_);
  H5Dclose(this->capability_dataset_);
  H5Gclose(this->group_capability_);
  H5Fclose(this->file_);
}

bool Hdf5Dataset::checkPath(std::string path)
{
  if (stat(path.c_str(), &st)!=0)
  {
    ROS_INFO("Path does not exist yet");
    //return false;
  }
}

void Hdf5Dataset::createPath(std::string path)
{
  ROS_INFO("Creating Directory");
  const int dir_err = mkdir(this->path_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if(1 == dir_err)
  {
    ROS_INFO("Error Creating Directory");
    exit(1);
  }
}

bool Hdf5Dataset::checkFileName(std::string filename)
{
  std::string ext = ".h5";
  if(filename.find(ext) == std::string::npos)
  {
    ROS_ERROR("Please provide an extension of .h5 It will make life easy");
    exit(1);
  }
}

bool Hdf5Dataset::saveCapMapsToDataset(Vector &capabilityData, float &resolution)
{
  if(!checkPath(this->path_))
  {
    createPath(this->path_);
  }
  const char *filepath = this->path_.c_str();
  const char *name = this->filename_.c_str();
  char fullpath[100];
  strcpy(fullpath, filepath);
  strcat(fullpath, name);
  ROS_INFO("Saving Capability Map %s", this->filename_.c_str());
  hid_t capability_dataspace;
  this->file_ = H5Fcreate(fullpath, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  this->group_capability_ = H5Gcreate(this->file_, "/Capability", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  const int SX = capabilityData.size();
  const int SY = 10;
  hsize_t dims2[2];               // dataset dimensions
  dims2[0] = SX;
  dims2[1] = SY;
  double dset2_data[SX][SY];
  for(int i=0;i<capabilityData.size();i++)
  {
    for(int j=0;j<capabilityData[i].size();j++)
    {
      dset2_data[i][j] = capabilityData[i][j];
    }
   }
  capability_dataspace = H5Screate_simple(2, dims2, NULL);
  this->capability_dataset_= H5Dcreate2(this->group_capability_,"capability_dataset",H5T_NATIVE_DOUBLE, capability_dataspace,
H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

  H5Dwrite(this->capability_dataset_, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT,
                   dset2_data);

 //Creating attribute
  hsize_t attr_dims;
  float attr_data[1];
  attr_data[0] = resolution;
  attr_dims =1;
  capability_dataspace = H5Screate_simple(1, &attr_dims, NULL);
  this->attr_ = H5Acreate2 (this->capability_dataset_, "Resolution", H5T_NATIVE_FLOAT, capability_dataspace, H5P_DEFAULT,
H5P_DEFAULT);
  H5Awrite(this->attr_, H5T_NATIVE_FLOAT, attr_data);

 //Closing all
  H5Sclose(capability_dataspace);
  close_cap();
  return 0;
}

bool Hdf5Dataset::saveReachMapsToDataset(MultiMap &Poses, Map &Spheres, float resolution)
{
  if(!checkPath(this->path_))
  {
    createPath(this->path_);
  }

  //Creating Multimap to Straight vector<vector<double> > //Can take this function to a new class
  std::vector< std::vector< double > > poseReach;
  for (MultiMap::iterator it = Poses.begin(); it != Poses.end(); ++it)
  {
    const std::vector<double>* sphere_coord    = it->first;
    const std::vector<double>* point_on_sphere = it->second;
    std::vector< double > poseAndSphere(10);
    //poseAndSphere.reserve( sphere_coord->size() + point_on_sphere->size());
    for (int i = 0; i < 3; i++)
      {
        poseAndSphere[i]=((*sphere_coord)[i]);
      }
    for (int j = 0; j < 7; j++)
      {
        poseAndSphere[3+j] = ((*point_on_sphere)[j]);
      }
    poseReach.push_back(poseAndSphere);
   }

  const char *filepath = this->path_.c_str();
  const char *name = this->filename_.c_str();
  char fullpath[100];
  strcpy(fullpath, filepath);
  strcat(fullpath, name);
  ROS_INFO("Saving map %s", this->filename_.c_str());
  this->file_ = H5Fcreate(fullpath, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  this->group_poses_ = H5Gcreate(this->file_, "/Poses", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  this->group_spheres_ = H5Gcreate(this->file_, "/Spheres", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  ROS_INFO("Saving poses in reachability map");
  const hsize_t ndims = 2;
  const hsize_t ncols = 10;

  int posSize = Poses.size();
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
  this->poses_dataset_ = H5Dcreate(this->group_poses_, "poses_dataset", H5T_NATIVE_FLOAT, file_space, H5P_DEFAULT, plist, H5P_DEFAULT);
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
  H5Dset_extent(this->poses_dataset_, dims);

  // Selecting hyperslab on the dataset
  file_space = H5Dget_space(this->poses_dataset_);
  hsize_t start[2] = {0, 0};
  hsize_t count[2] = {chunk_size, ncols};
  H5Sselect_hyperslab(file_space, H5S_SELECT_SET, start, NULL, count, NULL);

  // Writing buffer to the dataset
  H5Dwrite(this->poses_dataset_, H5T_NATIVE_FLOAT, mem_space, file_space, H5P_DEFAULT, buffer);

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
  H5Dset_extent(this->poses_dataset_, dims);
  // Selecting hyperslab
  file_space = H5Dget_space(this->poses_dataset_);
  start[0] = chunk_size;
  start[1] = 0;
  count[0] = posSize - chunk_size;
  count[1] = ncols;
  H5Sselect_hyperslab(file_space, H5S_SELECT_SET, start, NULL, count, NULL);

  // Writing buffer to dataset
  H5Dwrite(this->poses_dataset_, H5T_NATIVE_FLOAT, mem_space, file_space, H5P_DEFAULT, buffer);

  // Closing all the resources
  delete[] dset1_data;
  delete[] buffer;


  // Creating Sphere dataset
  ROS_INFO("Saving spheres in Reachability map");
  hid_t sphere_dataspace;
  const int SX = Spheres.size();
  const int SY = 4;

  hsize_t dims2[2];  // dataset dimensions
  dims2[0] = SX;
  dims2[1] = SY;
  double dset2_data[SX][SY];

  for (Map::iterator it =  Spheres.begin(); it !=Spheres.end(); ++it)
    {
      for (int j = 0; j < SY - 1; j++)
      {
        dset2_data[distance( Spheres.begin(), it)][j] = (*it->first)[j];
      }
      for (int j = 3; j < SY; j++)
      {
        dset2_data[distance( Spheres.begin(), it)][j] = it->second;
      }
    }
  sphere_dataspace = H5Screate_simple(2, dims2, NULL);
  this->sphere_dataset_ = H5Dcreate2(this->group_spheres_, "sphere_dataset", H5T_NATIVE_DOUBLE,
                                     sphere_dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(this->sphere_dataset_, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, dset2_data);

  // Creating attribute


  hsize_t attr_dims;
  float attr_data[1];
  attr_data[0] = resolution;
  attr_dims = 1;
  sphere_dataspace = H5Screate_simple(1, &attr_dims, NULL);
  this->attr_ = H5Acreate2(this->sphere_dataset_, "Resolution", H5T_NATIVE_FLOAT, sphere_dataspace,
                           H5P_DEFAULT, H5P_DEFAULT);
  H5Awrite(this->attr_, H5T_NATIVE_FLOAT, attr_data);
  //H5Aclose(this->attr_);

  // Closing all

  H5Sclose(sphere_dataspace);
  H5Sclose(file_space);
  H5Sclose(mem_space);
  close();
}

bool Hdf5Dataset::h5ToVectorCap(Vector &capabilityData)
{
  hsize_t dims_out[2], count[2], offset[2], dimsm[2];
  hid_t dataspace = H5Dget_space(this->capability_dataset_);
  int rank = H5Sget_simple_extent_ndims(dataspace);
  herr_t status_n = H5Sget_simple_extent_dims(dataspace, dims_out, NULL);
  herr_t status;
  offset[0] = 0;
  offset[1] = 0;
  count[0] = dims_out[0];
  count[1] = 10;
  double data_out[count[0]][count[1]];
  status = H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offset, NULL, count, NULL);
  dimsm[0] = count[0];
  dimsm[1] = count[1];
  hid_t memspace;
  memspace = H5Screate_simple(RANK_OUT, dimsm, NULL);
  status = H5Dread(this->capability_dataset_, H5T_NATIVE_DOUBLE, memspace, dataspace, H5P_DEFAULT, data_out);
  for(int i=0; i<count[0]; i++)
  {
    std::vector<double> capData(10);
    for(int j =0; j<10; j++)
    {
      capData[j] = data_out[i][j];
    }
    capabilityData.push_back(capData);
  }
  return 0;
}

bool Hdf5Dataset::h5ToMultiMapPosesAndSpheres(MultiMap& PoseCol, Map& SphereCol)
{
  //The process of creting typedef MultiMap is little bit tricky.
  //As the data are read from the h5 file as chuck of array, the retrieved sphere addresses comes different for every sphere
  //But that is not the case for our map structure. Every sphere with same coordinate has same address in h5
  //So to create the exact same Multimap and Map while maintaining the same strcture, we have to assign the spheres that have the same coordinates, with the same address
  //Otherwise when we will be loading the dataset for visualization or other taks such as inverse map, the comparison between the coordinates of sphere dataset and
  //coordinates of spheres in Poses dataset are going to fail.

  multiMap sphereAndPoses;
  map_D sps;
  h5ToMultiMapPoses(sphereAndPoses, sps);

  map_D SpCol;
  h5ToMultiMapSpheres(SpCol);

 for(map_D::iterator it=sps.begin(); it!=sps.end();++it)
  {
    //const std::vector<double>* sp = &(it->first);
    std::vector<double>* sp = new std::vector<double>(3);
    (*sp)[0] = (it->first)[0];
    (*sp)[1] = (it->first)[1];
    (*sp)[2] = (it->first)[2];
    multiMap::iterator it1;
    for( it1= sphereAndPoses.lower_bound(it->first); it1 != sphereAndPoses.upper_bound(it->first); ++it1)
    {
      std::vector<double>* ps = new std::vector<double>(7);
      (*ps)[0]=(it1->second[0]);
      (*ps)[1]=(it1->second[1]);
      (*ps)[2]=(it1->second[2]);
      (*ps)[3]=(it1->second[3]);
      (*ps)[4]=(it1->second[4]);
      (*ps)[5]=(it1->second[5]);
      (*ps)[6]=(it1->second[6]);
       PoseCol.insert(std::make_pair(sp, ps));
      }

    for (map_D::iterator it2 = SpCol.lower_bound(it->first); it2 !=SpCol.upper_bound(it->first); ++it2)
    {
      SphereCol.insert(std::make_pair(sp, it2->second));
     }
   }
return 0;
}

bool Hdf5Dataset::h5ToMultiMapPoses(multiMap &PoseCol)
{
  map_D sphereCol;
  h5ToMultiMapPoses(PoseCol, sphereCol);
}

bool Hdf5Dataset::h5ToMultiMapPoses(multiMap &PoseCol, map_D &SphereCol)
{
  hsize_t dims_out[2], count[2], offset[2];
  hid_t dataspace = H5Dget_space(this->poses_dataset_); /* dataspace handle */
  int rank = H5Sget_simple_extent_ndims(dataspace);
  herr_t status_n = H5Sget_simple_extent_dims(dataspace, dims_out, NULL);
  herr_t status;
  int chunk_size, chunk_itr;
  if (dims_out[0] % 10)
  {
    chunk_itr = 11;
  }
  else
  {
    chunk_itr = 10;
  }
  chunk_size = (dims_out[0] / 10);
  offset[0] = 0;

  for (int it = 0; it < chunk_itr; it++)
  {
    offset[1] = 0;
    if ((dims_out[0] - (chunk_size * it)) / chunk_size != 0)
    {
      count[0] = chunk_size;
      offset[0] = chunk_size * it;
    }
    else
    {
      count[0] = (dims_out[0] - (chunk_size * it));
      offset[0] = count[0];
    }
    count[1] = 10;

    double data_out[count[0]][count[1]];

    status = H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offset, NULL, count, NULL);
    hsize_t dimsm[2];
    dimsm[0] = count[0];
    dimsm[1] = count[1];
    hid_t memspace;
    memspace = H5Screate_simple(RANK_OUT, dimsm, NULL);
    status = H5Dread(this->poses_dataset_, H5T_NATIVE_DOUBLE, memspace, dataspace, H5P_DEFAULT, data_out);

    for(int i=0; i<count[0]; i++)
    {
      std::vector<double> sphereCenter(3);
      std::vector<double> Poses(7);
      for(int j=0; j<3;j++)
      {
        sphereCenter[j] = data_out[i][j];
        }
      for(int k=3;k<10; k++)
      {
        Poses[k-3] = data_out[i][k];
        }
      PoseCol.insert(std::make_pair(sphereCenter, Poses));
      SphereCol.insert(std::make_pair(sphereCenter, double(i)));
      }
  }
return 0;
}

bool Hdf5Dataset::h5ToMultiMapSpheres(map_D &SphereCol)
{
  hsize_t dims_out[2], count[2], offset[2], dimsm[2];
  hid_t dataspace = H5Dget_space(this->sphere_dataset_); // dataspace handle
  int rank = H5Sget_simple_extent_ndims(dataspace);
  herr_t status_n = H5Sget_simple_extent_dims(dataspace, dims_out, NULL);
  herr_t status;
  offset[0] = 0;
  offset[1] = 0;
  count[0] = dims_out[0];
  count[1] = 4;
  double data_out[count[0]][count[1]];
  status = H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offset, NULL, count, NULL);
  dimsm[0] = count[0];
  dimsm[1] = count[1];
  hid_t memspace;
  memspace = H5Screate_simple(RANK_OUT, dimsm, NULL);
  status = H5Dread(this->sphere_dataset_, H5T_NATIVE_DOUBLE, memspace, dataspace, H5P_DEFAULT, data_out);
  for (int i = 0; i < count[0]; i++)
  {
    std::vector< double > sphereCenter(3);
    double ri;
    for (int j = 0; j < 3; j++)
    {
      sphereCenter[j] = data_out[i][j];
    }
    for (int k = 3; k < 4; k++)
    {
      ri = data_out[i][k];
    }
    SphereCol.insert(std::pair< std::vector< double >, double >(sphereCenter, ri));
   }
  return 0;
}

bool Hdf5Dataset::h5ToResolution(float &resolution)
{
  resolution = this->res_;
  return 0;
}

bool Hdf5Dataset::loadMapsFromDataset(MultiMap &Poses, Map &Spheres, float &resolution)
{
  h5ToMultiMapPosesAndSpheres(Poses, Spheres);
  h5ToResolution(resolution);
  close();
  return 0;
}

bool Hdf5Dataset::loadMapsFromDataset(MultiMap &Poses, Map &Spheres)
{
  h5ToMultiMapPosesAndSpheres(Poses, Spheres);
  close();
  return 0;
}

bool Hdf5Dataset::loadMapsFromDataset(multiMap &Poses, map_D &Spheres)
{
  h5ToMultiMapPoses(Poses);
  h5ToMultiMapSpheres(Spheres);
  close();
  return 0;
}

bool Hdf5Dataset::loadMapsFromDataset(multiMap &Poses, map_D &Spheres, float &resolution)
{
  h5ToMultiMapPoses(Poses);
  h5ToMultiMapSpheres(Spheres);
  h5ToResolution(resolution);
  close();
  return 0;
}

bool Hdf5Dataset::loadCapMapFromDataset(Vector &capabilityData, float &resolution)
{
  h5ToVectorCap(capabilityData);
  h5ToResolution(resolution);
  close_cap();
  return 0;
}


};
