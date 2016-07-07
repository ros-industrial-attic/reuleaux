#include <ros/ros.h>

#include "map_creator/WorkSpace.h"


#include "H5Cpp.h"
#include <hdf5.h>

using namespace H5;
using namespace std;


#define POSES_DATASETNAME "poses_dataset"
#define SPHERE_DATASETNAME "sphere_dataset"
#define POSE_GROUPNAME "/Poses"
#define SPHERE_GROUPNAME "/Spheres"
#define RANK_OUT 2

class loadHD5{
public:

void h5ToMultiMapPoses(const hid_t dataset, multimap<vector<double>, vector<double> >& PoseColFilter)
{
    hsize_t dims_out[2], count[2], offset[2];
    hid_t dataspace = H5Dget_space (dataset);    /* dataspace handle */
    int rank      = H5Sget_simple_extent_ndims (dataspace);
    herr_t status_n  = H5Sget_simple_extent_dims (dataspace, dims_out, NULL);
    herr_t status;
    int chunk_size,  chunk_itr;
    if (dims_out[0]%10)
      {        
	chunk_itr=11;
	}
    else
      {
       	chunk_itr=10;
	}
    chunk_size=(dims_out[0]/10);
    offset[0] = 0;
    
    for(int it=0;it<chunk_itr;it++)
    {
      
      offset[1] = 0;
	 
      
      if ((dims_out[0]-(chunk_size*it))/chunk_size != 0){
	count[0] = chunk_size;
	offset[0] = chunk_size*it;
	
      }
      else{
	count[0] = (dims_out[0]-(chunk_size*it));
	offset[0] = count[0];
      }
      
      
      count[1]  = 10;
      
      double data_out[count[0]][count[1]];
	
      status = H5Sselect_hyperslab (dataspace, H5S_SELECT_SET, offset, NULL, 
                                  count, NULL);
	hsize_t dimsm[2]; 
	dimsm[0] = count[0];
    	dimsm[1] =  count[1];
	hid_t memspace;
	
        memspace = H5Screate_simple (RANK_OUT, dimsm, NULL);  

	status = H5Dread (dataset, H5T_NATIVE_DOUBLE, memspace, dataspace,
                      H5P_DEFAULT, data_out);

	
	for(int i=0;i<count[0];i++){

          vector<double> sphereCenter;
	  vector<double> Poses;
 	  for(int j=0;j<3;j++)
	  {
		 sphereCenter.push_back(data_out[i][j]);
		 }
	  
	  for(int k=3;k<10;k++)
	  {
		 Poses.push_back(data_out[i][k]);
		}
	    
	PoseColFilter.insert(pair<vector<double>, vector<double> >(sphereCenter, Poses));
	}
  }

}

void h5ToMultiMapSpheres(const hid_t dataset, multimap<vector<double>, double >& SphereCol){

    hsize_t dims_out[2], count[2], offset[2], dimsm[2];
    hid_t dataspace = H5Dget_space (dataset);    /* dataspace handle */
    int rank      = H5Sget_simple_extent_ndims (dataspace);
    herr_t status_n  = H5Sget_simple_extent_dims (dataspace, dims_out, NULL);
    herr_t status;
    offset[0] = 0;
    offset[1] = 0;
    count[0]  = dims_out[0];
    count[1]  = 4;
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
          vector<double> sphereCenter;
	  double ri;
 	  for(int j=0;j<3;j++)
	    {
		 sphereCenter.push_back(data_out[i][j]);
	    }
	  
	  for(int k=3;k<4;k++)
	    {
		 ri = data_out[i][k];
	    }
	    
	  SphereCol.insert(pair<vector<double>, double >(sphereCenter, ri));
	}
}

};


int main(int argc, char **argv)
{
    if (argc<2){
	ROS_ERROR_STREAM("Please provide the name of the reachability map. If you have not created it yet, Please create the map by running the create reachability map launch file in map_creator");
	return 1;
    } 
    else{
	
    ros::init(argc, argv, "workspace");
    ros::NodeHandle n;
    

    ros::Publisher workspace_pub = n.advertise<map_creator::WorkSpace>("reachability_map", 1);
    ros::Rate loop_rate(10);
  
    int count = 0;
    while (ros::ok())
    {   
	time_t startit,finish;
        time (&startit);
   	const char* FILE = argv[1];
        hid_t file, poses_group, poses_dataset, sphere_group, sphere_dataset, attr; 
	file = H5Fopen (FILE, H5F_ACC_RDONLY, H5P_DEFAULT);

//Poses dataset

	poses_group = H5Gopen (file, POSE_GROUPNAME, H5P_DEFAULT);
        poses_dataset = H5Dopen (poses_group, POSES_DATASETNAME, H5P_DEFAULT);

	multimap<vector<double>, vector<double> > PoseColFilter;
	loadHD5 hd5;
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
   workspace_pub.publish(ws);
    
    time (&finish);
    double dif = difftime (finish,startit);
    ROS_INFO ("Elasped time is %.2lf seconds.", dif );
    ROS_INFO ("Completed");
    ros::spinOnce();
    sleep(10);
    ++count;
 }
}
return 0;
}
