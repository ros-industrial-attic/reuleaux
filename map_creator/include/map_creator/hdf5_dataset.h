#ifndef HDF5_DATASET_H
#define HDF5_DATASET_H
#include "H5Cpp.h"
#include <hdf5.h>
#include <iostream>
#include <ros/ros.h>

namespace hdf5_dataset
{
class Hdf5Dataset
{
public:
  // Hdf5Dataset();

  //~Hdf5Dataset();

  void h5ToMultiMapPoses(const hid_t dataset, std::multimap< std::vector< double >, std::vector< double > >& PoseColFilter);

  void h5ToMultiMapSpheres(const hid_t dataset, std::multimap< std::vector< double >, double >& SphereCol);

  void h5ToMultiMapCap(const hid_t dataset, std::multimap< std::vector< double >, std::vector< double > >& sphereColor);
};

}  // namespace hdf5_dataset
#endif  // HDF5_DATASET_H
