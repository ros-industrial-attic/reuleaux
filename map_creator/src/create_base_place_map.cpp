//The inverse reachability map depends on the reachability map. It is an inversion of the poses to the base location

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <algorithm>
#include <ros/ros.h>
#include <ros/package.h>

#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>

#include <map_creator/sphere_discretization.h>
#include <map_creator/kinematics.h>
#include <map_creator/hdf5_dataset.h>
#include <map_creator/WorkSpace.h>
#include <map>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include "geometry_msgs/PoseArray.h"

#include <sys/types.h> 
#include <sys/stat.h> 
#include <ctime>

#include "H5Cpp.h"
#include <hdf5.h>
#include <string>
#include <time.h>

using namespace H5;
using namespace octomap;
using namespace std;
using namespace octomath;
using namespace sphere_discretization;
using namespace kinematics;
using namespace hdf5_dataset;
struct stat st;



void associatePose(multimap<vector<double>, vector<double> >& baseTrnsCol, const vector<geometry_msgs::Pose> grasp_poses, const multimap<vector<double>, vector<double> > PoseColFilter, const float resolution)
{
   
    unsigned char maxDepth = 16;
    float size_of_box =1.5;
    SphereDiscretization sd;
    point3d origin=point3d(0,0,0);
    OcTree* tree = sd.generateBoxTree(origin, size_of_box, resolution);
    vector<point3d> spCenter;
    for (OcTree::leaf_iterator it=tree->begin_leafs(maxDepth), end=tree->end_leafs();it !=end;++it){
      spCenter.push_back(it.getCoordinate());
      }


    multimap<vector<float>, vector<float> > trns_col;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0;i<grasp_poses.size();++i)
    {
      tf2::Vector3 grasp_vec(grasp_poses[i].position.x, grasp_poses[i].position.y, grasp_poses[i].position.z);
      tf2::Quaternion grasp_quat(grasp_poses[i].orientation.x, grasp_poses[i].orientation.y, grasp_poses[i].orientation.z, grasp_poses[i].orientation.w);
      grasp_quat.normalize();
      tf2::Transform grasp_trns;
      grasp_trns.setOrigin(grasp_vec);
      grasp_trns.setRotation(grasp_quat);

      for (multimap<vector<double>, vector<double> >::const_iterator it = PoseColFilter.begin();it != PoseColFilter.end();++it)
      {
        tf2::Vector3 vec(it->second[0], it->second[1], it->second[2]); 
	tf2::Quaternion quat(it->second[3], it->second[4], it->second[5], it->second[6]);
	tf2::Transform trns;
	trns.setOrigin(vec);
        trns.setRotation(quat);
	
	tf2::Transform new_trns;
	//new_trns = trns * grasp_trns;
	new_trns = grasp_trns * trns;

        

        tf2::Vector3 new_trans_vec;
	tf2::Quaternion new_trans_quat;
        new_trans_vec = new_trns.getOrigin();
	new_trans_quat = new_trns.getRotation();
	new_trans_quat.normalize();

	vector<float> position;
	position.push_back(new_trans_vec[0]);
	position.push_back(new_trans_vec[1]);
	position.push_back(new_trans_vec[2]);
	vector<float> orientation;
	orientation.push_back(new_trans_quat[0]);
	orientation.push_back(new_trans_quat[1]);
	orientation.push_back(new_trans_quat[2]);
	orientation.push_back(new_trans_quat[3]);
	trns_col.insert(pair<vector<float>, vector<float> >(position, orientation));

	pcl::PointXYZ point;
	point.x = new_trans_vec[0];
	point.y = new_trans_vec[1];
	point.z = new_trans_vec[2];
	cloud->push_back(point);
      }
    }
   
    //cout<<"Num of poses after creating cloud: "<<trns_col.size()<<endl;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();
    
    for(int i=0;i<spCenter.size();i++)
    { 
      pcl::PointXYZ searchPoint;
      searchPoint.x = spCenter[i].x();
      searchPoint.y = spCenter[i].y();
      searchPoint.z = spCenter[i].z();
      // Neighbors within voxel search

      std::vector<int> pointIdxVec;
      octree.voxelSearch (searchPoint, pointIdxVec);
      if(pointIdxVec.size()>0)
        {
	  for (size_t j=0;j<pointIdxVec.size();++j)
	  {
	    std::vector<float> base_pos;
	    base_pos.push_back(cloud->points[pointIdxVec[j]].x);
	    base_pos.push_back(cloud->points[pointIdxVec[j]].y);
	    base_pos.push_back(cloud->points[pointIdxVec[j]].z);
	    multimap<vector<float>, vector<float> >::iterator it1;
	    for(it1 = trns_col.lower_bound(base_pos); it1 !=trns_col.upper_bound(base_pos); ++it1){
		vector<double> base_pose;
	        base_pose.push_back(base_pos[0]);
	        base_pose.push_back(base_pos[1]);
	        base_pose.push_back(base_pos[2]);
	        base_pose.push_back(it1->second[0]); 
	        base_pose.push_back(it1->second[1]); 
	        base_pose.push_back(it1->second[2]); 
	        base_pose.push_back(it1->second[3]); 

	        vector<double>base_sphere;
	        base_sphere.push_back(searchPoint.x);
	        base_sphere.push_back(searchPoint.y);
	        base_sphere.push_back(searchPoint.z);
		baseTrnsCol.insert(pair<vector<double>, vector<double> >(base_sphere, base_pose));
	    }
	  }
        }
    }
}





int main(int argc, char **argv)
{
  if (argc<2){
	ROS_ERROR_STREAM("Please provide the name of the reachability map. If you have not created it yet, Please create the map by running the create reachability map node in map_creator package");
	return 1;
    } 
  else{
  ros::init(argc, argv, "base_place");
  ros::NodeHandle n;
  time_t startit,finish;
  time (&startit);
  ros::Rate loop_rate(10);
  

  int count = 0;
  while (ros::ok())
    {
     
     const char* FILE = argv[1];
     hid_t file, poses_group, poses_dataset, sphere_group, sphere_dataset, attr; 
     file = H5Fopen (FILE, H5F_ACC_RDONLY, H5P_DEFAULT);

//Poses dataset

     poses_group = H5Gopen (file, "/Poses", H5P_DEFAULT);
     poses_dataset = H5Dopen (poses_group, "poses_dataset", H5P_DEFAULT);

     multimap<vector<double>, vector<double> > PoseColFilter;
     Hdf5Dataset hd5;
     hd5.h5ToMultiMapPoses(poses_dataset, PoseColFilter);

//Sphere dataset
     sphere_group = H5Gopen (file, "/Spheres", H5P_DEFAULT);
     sphere_dataset = H5Dopen (sphere_group, "sphere_dataset", H5P_DEFAULT);
     multimap<vector<double>, double > SphereCol;
     hd5.h5ToMultiMapSpheres(sphere_dataset, SphereCol);

//Resolution Attribute
     float res;
     attr = H5Aopen(sphere_dataset,"Resolution",H5P_DEFAULT);
     herr_t ret = H5Aread(attr, H5T_NATIVE_FLOAT, &res);

//Starting to create the Inverse Reachability map. The resolution will be the same as the reachability map
   

    SphereDiscretization sd;
    multimap<vector<double>, vector<double> >baseTrnsCol;
    vector<geometry_msgs::Pose> grasp_poses;
    tf2::Quaternion grasp_quat1(0.5, 0.5, 0.5, 1); 
    grasp_quat1.normalize();
    tf2::Quaternion grasp_quat2(0, 0, 0, 1); 
    grasp_quat2.normalize();
    tf2::Quaternion grasp_quat3(0, 0.55, 0.28, 1); 
    grasp_quat3.normalize();   




    geometry_msgs::Pose grasp_pose1;
    grasp_pose1.position.x = 0;
    grasp_pose1.position.y = 0;
    grasp_pose1.position.z = 1;
    grasp_pose1.orientation.x = grasp_quat1[0];
    grasp_pose1.orientation.y = grasp_quat1[1];
    grasp_pose1.orientation.z = grasp_quat1[2];
    grasp_pose1.orientation.w = grasp_quat1[3];
    grasp_poses.push_back(grasp_pose1);   


    geometry_msgs::Pose grasp_pose2;
    grasp_pose2.position.x = 0.55;
    grasp_pose2.position.y = 0;
    grasp_pose2.position.z = 0.55;
    grasp_pose2.orientation.x = grasp_quat2[0];
    grasp_pose2.orientation.y = grasp_quat2[1];
    grasp_pose2.orientation.z = grasp_quat2[2];
    grasp_pose2.orientation.w = grasp_quat2[3];
    grasp_poses.push_back(grasp_pose2);   

    geometry_msgs::Pose grasp_pose3;
    grasp_pose3.position.x = 0.99;
    grasp_pose3.position.y = 0.23;
    grasp_pose3.position.z = 0.34;
    grasp_pose3.orientation.x = grasp_quat3[0];
    grasp_pose3.orientation.y = grasp_quat3[1];
    grasp_pose3.orientation.z = grasp_quat3[2];
    grasp_pose3.orientation.w = grasp_quat3[3];
    grasp_poses.push_back(grasp_pose3);    

     


    //ROS_INFO("Number of poses in IRM: %lu",PoseColFilter.size());
    associatePose(baseTrnsCol, grasp_poses, PoseColFilter, res);
    
    //cout<<"Total number of Poses: "<<baseTrnsCol.size()<<endl;
//Normalization for inverse Reachability Index
    
    vector<int> poseCount;
    
    for (multimap<vector<double>, vector<double> > ::iterator it = baseTrnsCol.begin();it != baseTrnsCol.end();++it)
    {
      int num = baseTrnsCol.count(it->first);
      poseCount.push_back(num);
    }
    
   vector<int>::const_iterator it;
   it = max_element(poseCount.begin(), poseCount.end());
   int max_number =*it;
   //cout<<"Maximum number of poses in a voxel: "<<max_number<<endl;
   

   it = min_element(poseCount.begin(), poseCount.end());
   int min_number =*it;
   //cout<<"Minimum number of poses in a voxel: "<<min_number<<endl;
   

//Here we dont need to save all the poses again. In the previous step we can take all the sphere centers in a map with associated number of poses. In this step we can just normalize the Inv reachability score for individual spheres and not saving all the poses again in the poseReach vector.

    vector<vector<double> > poseReach;
    map<vector<double>, double>  sphereColor;
    
    for (multimap<vector<double>, vector<double> > ::iterator it = baseTrnsCol.begin();it != baseTrnsCol.end();++it)
    {
	float d = ((float(baseTrnsCol.count(it->first))-min_number)/(max_number-min_number))*100;
	if (d>1){
	sphereColor.insert(pair<vector<double>, double>(it->first,double(d)));
	
        }
	
	vector<double> poseAndSphere;
        for(int i=0;i<(it->first).size();i++){
	    poseAndSphere.push_back((it->first)[i]);
	    }
	for(int j=0;j<(it->second).size();j++){
	    poseAndSphere.push_back((it->second)[j]);
	    }
        
	poseReach.push_back(poseAndSphere);
    }
    
    //ROS_INFO("Numer of Spheres in IRM: %lu",SphereCol.size());
    //ROS_INFO("Numer of Spheres in Transformed IRM: %lu",sphereColor.size());
    //ROS_INFO("Numer of Poses in Transformed IRM: %lu",poseReach.size());
    //ROS_INFO("All the poses have Processed. Now saving data to a inverse Reachability Map.");

   


//Creating maps now
     string path(ros::package::getPath("map_creator")+"/maps/");
    if (stat(path.c_str(),&st)!=0)
	ROS_INFO("Path does not exist. Creating folder for maps");
	const int dir_err = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (1 == dir_err)
	{
    		ROS_INFO("Error creating directory");
    		exit(1);
	}

//TODO the filename will be an argument
//If the user does not provide a filename, then the default name will be saved

    time_t currentTime;
    struct tm *localTime;
    time( &currentTime );        
    localTime = localtime( &currentTime ); 
   
    int Day    = localTime->tm_mday;
    int Month  = localTime->tm_mon + 1;
    int Year   = localTime->tm_year + 1900;
    int Hour   = localTime->tm_hour;
    int Min    = localTime->tm_min;
    int Sec    = localTime->tm_sec;

//Creating all the file and group ids and the default file name 
 
    string filename;
//    filename=string(k.getRobotName())+"_"+boost::lexical_cast<std::string>(Hour)+":"+boost::lexical_cast<std::string>(Min)+"_"+boost::lexical_cast<std::string>(Month)+":"+boost::lexical_cast<std::string>(Day)+":"+boost::lexical_cast<std::string>(Year)+"_"+"r"+str( boost::format("%d") % resolution)+"_"+"sd"+"_"+"rot"+"_"+"reachability"+"."+"h5";

//The filename is shortened for now for testing purpose.
    Kinematics k;
    filename=string(k.getRobotName())+"_"+"r"+str( boost::format("%d") % res)+"_"+"Base_place"+"."+"h5";

    const char * filepath = path.c_str();
    const char * name = filename.c_str();
    char fullpath[100];
    strcpy(fullpath,filepath);
    strcat(fullpath,name);
    ROS_INFO("Saving map %s",filename.c_str());
    hid_t file_IRM, poses, spheres;

    file_IRM = H5Fcreate(fullpath, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    poses = H5Gcreate(file_IRM,"/Poses",H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    spheres = H5Gcreate(file_IRM,"/Spheres",H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);

    const hsize_t ndims = 2;
    const hsize_t ncols = 10;
    
    int posSize=baseTrnsCol.size();
    int chunk_size;
    int PY =10;
    if (posSize%2){
        chunk_size=(posSize/2)+1;
        	}
    else{
         chunk_size=(posSize/2);
    }
//Create Dataspace 
    hsize_t dims[ndims] = {0, ncols};	//Starting with an empty buffer
    hsize_t max_dims[ndims] = {H5S_UNLIMITED, ncols}; //Creating dataspace
    hid_t file_space = H5Screate_simple(ndims, dims, max_dims);  

//Create Dataset Property list 
    hid_t plist = H5Pcreate(H5P_DATASET_CREATE);
    H5Pset_layout(plist, H5D_CHUNKED);
    hsize_t chunk_dims[ndims] = {chunk_size, ncols};
    H5Pset_chunk(plist, ndims, chunk_dims);

//Create the datset
    hid_t dset = H5Dcreate(poses, "poses_dataset", H5T_NATIVE_FLOAT, file_space, H5P_DEFAULT, plist, H5P_DEFAULT);

//Closing resources
    H5Pclose(plist);
    H5Sclose(file_space);

//Creating the first buffer
    hsize_t nlines = chunk_size;
    float *buffer = new float[nlines * ncols];
    float **dset1_data = new float*[nlines];
    for (hsize_t i = 0; i < nlines; ++i){
        dset1_data[i] = &buffer[i * ncols];
    }

//Data for the first chunk     
    for (int i=0;i<chunk_size;i++){
	for(int j=0;j<PY;j++){
	   dset1_data[i][j]=poseReach[i][j];
	   }
     }

//Memory dataspace indicating size of the buffer    
    dims[0] = chunk_size;
    dims[1] = ncols;
    hid_t mem_space = H5Screate_simple(ndims, dims, NULL);
    

//Extending dataset
    dims[0] = chunk_size;
    dims[1] = ncols;
    H5Dset_extent(dset, dims);
    

//Selecting hyperslab on the dataset
    file_space = H5Dget_space(dset);
    hsize_t start[2] = {0, 0};
    hsize_t count[2] = {chunk_size, ncols};
    H5Sselect_hyperslab(file_space, H5S_SELECT_SET, start, NULL, count, NULL);
    
//Writing buffer to the dataset
    H5Dwrite(dset, H5T_NATIVE_FLOAT, mem_space, file_space, H5P_DEFAULT, buffer);
    

//Closing file dataspace
    H5Sclose(file_space);

//Data for the Second chunk	
    for (int i=chunk_size;i<posSize;i++){
	for(int j=0;j<PY;j++){
	   
	   dset1_data[i-chunk_size][j]=poseReach[i][j];
	   }
     } 
    
   
//Resizing new memory dataspace indicating new size of the buffer
    dims[0] = posSize-chunk_size;
    dims[1] = ncols;
    H5Sset_extent_simple(mem_space, ndims, dims, NULL);
    

//Extend dataset
    dims[0] = posSize;
    dims[1] = ncols;
    H5Dset_extent(dset, dims);
    

//Selecting hyperslab
    file_space = H5Dget_space(dset);
    start[0] = chunk_size;
    start[1] = 0;
    count[0] = posSize-chunk_size;
    count[1] = ncols;
    H5Sselect_hyperslab(file_space, H5S_SELECT_SET, start, NULL, count, NULL);
    

//Writing buffer to dataset
    H5Dwrite(dset, H5T_NATIVE_FLOAT, mem_space, file_space, H5P_DEFAULT, buffer);
   
//Closing all the resources
    delete[] dset1_data;
    delete[] buffer;
    H5Sclose(file_space);
    H5Sclose(mem_space);
    H5Dclose(dset);
    H5Gclose(poses);
        
    
//This part of the map not be needed for future work, as the sphere data is already stored in the poses dataset
//Creating Sphere dataset
    hid_t reachability_sphere_dataset, reachability_sphere_dataspace;
    const int SX = sphereColor.size();                     
    const int SY = 4;
    
    hsize_t dims2[2];               // dataset dimensions
    dims2[0] = SX;
    dims2[1] = SY;
    double dset2_data[SX][SY];

    for (map<vector<double>, double> ::iterator it = sphereColor.begin();it != sphereColor.end();++it){
        for(int j=0;j<SY-1;j++){
	    dset2_data[distance(sphereColor.begin(), it)][j]=it->first[j];
	    }
        for(int j=3;j<SY;j++){
	    dset2_data[distance(sphereColor.begin(), it)][j]=it->second;
	    }
    }
    reachability_sphere_dataspace = H5Screate_simple(2, dims2, NULL);
    reachability_sphere_dataset = H5Dcreate2(spheres,"sphere_dataset",H5T_NATIVE_DOUBLE, reachability_sphere_dataspace,  H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    
    H5Dwrite(reachability_sphere_dataset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, dset2_data);

    
    
//Creating attribute
    
    hid_t attr_id;
    hsize_t attr_dims;
    float attr_data[1];
    attr_data[0] = res;
    attr_dims =1;
    reachability_sphere_dataspace = H5Screate_simple(1, &attr_dims, NULL);
    attr_id = H5Acreate2 (reachability_sphere_dataset, "Resolution", H5T_NATIVE_FLOAT, reachability_sphere_dataspace, H5P_DEFAULT, H5P_DEFAULT);
    H5Awrite(attr_id, H5T_NATIVE_FLOAT, attr_data);
    H5Aclose(attr_id);
    

//Closing all

    H5Sclose(reachability_sphere_dataspace);
    H5Dclose(reachability_sphere_dataset);
    H5Gclose(spheres);
    

    H5Fclose(file_IRM);



    time (&finish);
    double dif = difftime (finish,startit);
    ROS_INFO ("Elasped time is %.2lf seconds.", dif );
    ROS_INFO ("Completed");
    ros::spinOnce();
      //sleep(10000);
    return 1;
    loop_rate.sleep();
    count;
    }
  }
return 0;
}
