#include <ros/ros.h>

#include "map_creator/capability.h"


#include "H5Cpp.h"
#include <hdf5.h>

using namespace H5;
using namespace std;

#define CAP_DATASETNAME "capability_dataset"
#define CAP_GROUPNAME "/Capability"
#define RANK_OUT 2

class loadHD5{
public:

void h5ToMultiMapCap(const hid_t dataset, multimap<vector<double>, vector<double> >& sphereColor){

    hsize_t dims_out[2], count[2], offset[2], dimsm[2];
    hid_t dataspace = H5Dget_space (dataset);    /* dataspace handle */
    int rank      = H5Sget_simple_extent_ndims (dataspace);
    herr_t status_n  = H5Sget_simple_extent_dims (dataspace, dims_out, NULL);
    herr_t status;
    offset[0] = 0;
    offset[1] = 0;
    count[0]  = dims_out[0];
    count[1]  = 10;
    double data_out[count[0]][count[1]];
    status = H5Sselect_hyperslab (dataspace, H5S_SELECT_SET, offset, NULL, 
                                  count, NULL);
    dimsm[0] = count[0];
    dimsm[1] =  count[1];
    hid_t memspace;
    memspace = H5Screate_simple (RANK_OUT, dimsm, NULL);  
    status = H5Dread (dataset, H5T_NATIVE_DOUBLE, memspace, dataspace,
                      H5P_DEFAULT, data_out);
    for(int i=0;i<count[0];i++){
	vector<double> posAndQuat;
	vector<double> enumRIAngle;
        for(int j=0;j<2;j++)
	    {
		 enumRIAngle.push_back(data_out[i][j]);
	    }
	for(int j=2;j<9;j++)
	    {
		 posAndQuat.push_back(data_out[i][j]);
	    }
        for(int j=9;j<10;j++)
	    {
		 enumRIAngle.push_back(data_out[i][j]);
	    }
        
        sphereColor.insert(pair<vector<double>, vector<double> >(posAndQuat, enumRIAngle));
    }
  }
};


int main(int argc, char **argv)
{
    if (argc<2){
	ROS_ERROR_STREAM("Please provide the name of the Capability map. If you have not created it yet, Please create the map by running the create cpability map launch file in map_creator");
	return 1;
    } 
    else{
	
    ros::init(argc, argv, "workspace");
    ros::NodeHandle n;
    bool latchOn = 1;
    ros::Publisher workspace_pub = n.advertise<map_creator::capability>("capability_map", 1);
    
    ros::Rate loop_rate(10);
  
      
      time_t startit,finish;
      time (&startit);
      const char* FILE = argv[1];
      hid_t file, cap_group, cap_dataset, attr; 
      file = H5Fopen (FILE, H5F_ACC_RDONLY, H5P_DEFAULT);

//capability dataset

      cap_group = H5Gopen (file, CAP_GROUPNAME, H5P_DEFAULT);
      cap_dataset = H5Dopen (cap_group, CAP_DATASETNAME, H5P_DEFAULT);
      multimap<vector<double>, vector<double> > sphereColor;
      loadHD5 hd5;
      hd5.h5ToMultiMapCap(cap_dataset, sphereColor);


//Resolution Attribute
      float res;
      attr = H5Aopen(cap_dataset,"Resolution",H5P_DEFAULT);
      herr_t ret = H5Aread(attr, H5T_NATIVE_FLOAT, &res);


//Closing resources
      H5Aclose(attr);
      H5Dclose(cap_dataset);
      H5Gclose(cap_group);
      H5Fclose(file);
	

//Creating messages

     

      map_creator::capability cp;
      cp.header.stamp = ros::Time::now();
      cp.header.frame_id = "/base_link";
      cp.resolution = res;

      for(multimap<vector<double>, vector<double> >::iterator it = sphereColor.begin();it !=sphereColor.end(); ++it){
      
	map_creator::capShape cpSp;
        cpSp.identifier = it->second[0];
        cpSp.ri = it->second[1];
        cpSp.angleSFE = it->second[2];
        cpSp.pose.position.x = it->first[0];
        cpSp.pose.position.y = it->first[1];
	cpSp.pose.position.z = it->first[2];
	cpSp.pose.orientation.x = it->first[3];
	cpSp.pose.orientation.y = it->first[4];
	cpSp.pose.orientation.z = it->first[5];
	cpSp.pose.orientation.w = it->first[6];
        cp.capShapes.push_back(cpSp);
        }
       
       int count = 0;
       
    while (ros::ok())
    { 
       workspace_pub.publish(cp);
       ros::spinOnce();
       sleep(1);
       ++count;
      }
   
  }
return 0;
}

