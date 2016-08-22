#include <ros/ros.h>

#include "map_creator/capability.h"
#include <map_creator/hdf5_dataset.h>

#include "H5Cpp.h"
#include <hdf5.h>

using namespace H5;
using namespace std;
using namespace hdf5_dataset;

#define CAP_DATASETNAME "capability_dataset"
#define CAP_GROUPNAME "/Capability"
#define RANK_OUT 2



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
      Hdf5Dataset hd5;
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

