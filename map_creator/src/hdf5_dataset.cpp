#include <map_creator/hdf5_dataset.h>

#define RANK_OUT 2

namespace hdf5_dataset
{
// Hdf5Dataset::Hdf5Dataset(){}

void Hdf5Dataset::h5ToMultiMapPoses(const hid_t dataset, std::multimap< std::vector< double >, std::vector< double > >& PoseColFilter)
{
  hsize_t dims_out[2], count[2], offset[2];
  hid_t dataspace = H5Dget_space(dataset); /* dataspace handle */
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
    status = H5Dread(dataset, H5T_NATIVE_DOUBLE, memspace, dataspace, H5P_DEFAULT, data_out);
    for (int i = 0; i < count[0]; i++)
    {
      std::vector< double > sphereCenter;
      std::vector< double > Poses;
      for (int j = 0; j < 3; j++)
      {
        sphereCenter.push_back(data_out[i][j]);
      }

      for (int k = 3; k < 10; k++)
      {
        Poses.push_back(data_out[i][k]);
      }

      PoseColFilter.insert(std::pair< std::vector< double >, std::vector< double > >(sphereCenter, Poses));
    }
  }
}

void Hdf5Dataset::h5ToMultiMapSpheres(const hid_t dataset, std::multimap< std::vector< double >, double >& SphereCol)
{
  hsize_t dims_out[2], count[2], offset[2], dimsm[2];
  hid_t dataspace = H5Dget_space(dataset); /* dataspace handle */
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
  status = H5Dread(dataset, H5T_NATIVE_DOUBLE, memspace, dataspace, H5P_DEFAULT, data_out);

  for (int i = 0; i < count[0]; i++)
  {
    std::vector< double > sphereCenter;
    double ri;
    for (int j = 0; j < 3; j++)
    {
      sphereCenter.push_back(data_out[i][j]);
    }

    for (int k = 3; k < 4; k++)
    {
      ri = data_out[i][k];
    }

    SphereCol.insert(std::pair< std::vector< double >, double >(sphereCenter, ri));
  }
}

void Hdf5Dataset::h5ToMultiMapCap(const hid_t dataset, std::multimap< std::vector< double >, std::vector< double > >& sphereColor)
{
  hsize_t dims_out[2], count[2], offset[2], dimsm[2];
  hid_t dataspace = H5Dget_space(dataset); /* dataspace handle */
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
  status = H5Dread(dataset, H5T_NATIVE_DOUBLE, memspace, dataspace, H5P_DEFAULT, data_out);
  for (int i = 0; i < count[0]; i++)
  {
    std::vector< double > posAndQuat;
    std::vector< double > enumRIAngle;
    for (int j = 0; j < 2; j++)
    {
      enumRIAngle.push_back(data_out[i][j]);
    }
    for (int j = 2; j < 9; j++)
    {
      posAndQuat.push_back(data_out[i][j]);
    }
    for (int j = 9; j < 10; j++)
    {
      enumRIAngle.push_back(data_out[i][j]);
    }

    sphereColor.insert(std::pair< std::vector< double >, std::vector< double > >(posAndQuat, enumRIAngle));
  }
}
};
