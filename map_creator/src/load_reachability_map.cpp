#include <ros/ros.h>

#include "map_creator/WorkSpace.h"
#include <map_creator/hdf5_dataset.h>

#include "H5Cpp.h"
#include <hdf5.h>

using namespace H5;
using namespace std;
using namespace hdf5_dataset;


#define POSES_DATASETNAME "poses_dataset"
#define SPHERE_DATASETNAME "sphere_dataset"
#define POSE_GROUPNAME "/Poses"
#define SPHERE_GROUPNAME "/Spheres"



int main(int argc, char **argv)
{
    if (argc<2){
	ROS_ERROR_STREAM("Please provide the name of the reachability map. If you have not created it yet, Please create the map by running the create reachability map launch file in map_creator");
	return 1;
    } 
    else{
	
    ros::init(argc, argv, "workspace");
    ros::NodeHandle n;
    
//TODO: It can be published as a latched topic. So the whole message will be published just once and stay on the topic
    ros::Publisher workspace_pub = n.advertise<map_creator::WorkSpace>("reachability_map", 1);
    //bool latchOn = 1;
    //ros::Publisher workspace_pub = n.advertise<map_creator::WorkSpace>("reachability_map", 1, latchOn);
    ros::Rate loop_rate(10);
  
    int count = 0;
      
	time_t startit,finish;
        time (&startit);
   	const char* FILE = argv[1];
        hid_t file, poses_group, poses_dataset, sphere_group, sphere_dataset, attr; 
	file = H5Fopen (FILE, H5F_ACC_RDONLY, H5P_DEFAULT);

//Poses dataset

	poses_group = H5Gopen (file, POSE_GROUPNAME, H5P_DEFAULT);
        poses_dataset = H5Dopen (poses_group, POSES_DATASETNAME, H5P_DEFAULT);

	multimap<vector<double>, vector<double> > PoseColFilter;
	Hdf5Dataset hd5;
	hd5.h5ToMultiMapPoses(poses_dataset, PoseColFilter);

//Sphere dataset
	sphere_group = H5Gopen (file, SPHERE_GROUPNAME, H5P_DEFAULT);
	sphere_dataset = H5Dopen (sphere_group, SPHERE_DATASETNAME, H5P_DEFAULT);

        multimap<vector<double>, double > SphereCol;
	hd5.h5ToMultiMapSpheres(sphere_dataset, SphereCol);


	
//Resolution Attribute
	float res;
	attr = H5Aopen(sphere_dataset,"Resolution",H5P_DEFAULT);
	herr_t ret = H5Aread(attr, H5T_NATIVE_FLOAT, &res);


//Closing resources
	H5Aclose(attr);
        H5Dclose(poses_dataset);
	H5Dclose(sphere_dataset);
	H5Gclose(poses_group);
	H5Gclose(sphere_group);
        H5Fclose(file);
	

//Creating messages
	

	map_creator::WorkSpace ws;
	ws.header.stamp = ros::Time::now();
        ws.header.frame_id = "/base_link";
	ws.resolution = res;
	
	for (multimap<vector<double>, double >::iterator it = SphereCol.begin();it != SphereCol.end();++it){
	  map_creator::WsSphere wss;
	  wss.point.x = it->first[0];
	  wss.point.y = it->first[1];
	  wss.point.z = it->first[2];
	  wss.ri = it->second;

	  multimap<vector<double>, vector<double> >::iterator it1;
	  for(it1 = PoseColFilter.lower_bound(it->first); it1 !=PoseColFilter.upper_bound(it->first); ++it1){
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



   while (ros::ok())
    { 
     workspace_pub.publish(ws);
    
     ros::spinOnce();
     sleep(5);
     ++count;
 }
}
return 0;
}
