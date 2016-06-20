

#include<ros/ros.h>
#include <ros/package.h>
#include "reuleaux/WsColor.h"
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <reuleaux/sphere_discretization.h>
#include <reuleaux/kinematics.h>
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

struct stat st;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "capability_map");
    ros::NodeHandle n;
    time_t startit,finish;
    time (&startit);
    ros::Publisher workspace_pub = n.advertise<reuleaux::WsColor>("workspace", 10);

    //ros::Publisher principal_component = n.advertise<geometry_msgs::Pose>("pc", 1);

   
  
  
  ros::Rate loop_rate(10);
  
  int count = 0;
  while (ros::ok())
  {
      
    float HI=-1.5, LO=1.5;
    unsigned char maxDepth = 16;
    unsigned char minDepth = 0;
//A box of radius 1 is created. It will be the size of the robot+1.5. Then the box is discretized by voxels of specified resolution 
//TODO resolution will be user argument
//The center of every voxels are stored in a vector
    
    SphereDiscretization sd;
    float r=1;
    float resolution = 0.5;
    point3d origin=point3d(0,0,0); //This point will be the base of the robot
    OcTree* tree=sd.generateBoxTree(origin, r, resolution);
    std::vector<point3d> newData;
    ROS_INFO("Creating the box and discretizing with resolution: %f",resolution);
    for (OcTree::leaf_iterator it=tree->begin_leafs(maxDepth), end=tree->end_leafs();it !=end;++it){
	
	newData.push_back(it.getCoordinate());
		
 	}
    ROS_INFO("Total no of spheres now: %lu",newData.size());
    ROS_INFO("Please hold ON. Spheres are discretized and all of the poses are checked for Ik solutions. May take some time");

    float radius=resolution;
    
    
    multimap<vector<double>, vector<double> > PoseCol;
    for (int i=0;i<newData.size();i++){
	
	vector<geometry_msgs::Pose> pose;
	vector<double> sphere_coord;
	sd.convertPointToVector(newData[i],sphere_coord);
        
	pose=sd.make_sphere_poses(newData[i],radius);
	for(int j=0;j<pose.size();j++){
		
		vector<double> point_on_sphere;
   		sd.convertPoseToVector(pose[j],point_on_sphere);
        	
		
   	
	PoseCol.insert(pair<vector<double>, vector<double> >(point_on_sphere,sphere_coord));
		}
	}
    
//Every pose is checked for IK solutions. The reachable poses and the their corresponsing joint solutions are stored. Only the First joint solution is stored. We may need this solutions in the future. Otherwise we can show the robot dancing with the joint solutions in a parallel thread
//TODO Support for more than 6DOF robots needs to be implemented.


    Kinematics k;
    multimap<vector<double>, vector<double> > PoseColFilter;
    multimap<vector<double>, vector<double> > PoseCol2;
    vector<vector<double> > ikSolutions;
    for (multimap<vector<double>, vector<double> >::iterator it = PoseCol.begin();it != PoseCol.end();++it){
        std::vector<double> joints;
	joints.resize(6);
	PoseCol2.insert(pair<vector<double>, vector<double> >(it->second,it->first));
	if (k.isIKSuccess(it->first,joints)){
		PoseColFilter.insert(pair<vector<double>, vector<double> >(it->second,it->first));
	        ikSolutions.push_back(joints);
		
	    }
    }

    ROS_INFO("Total number of poses: %lu",PoseCol.size());
    ROS_INFO("Total number of reachable poses: %lu",PoseColFilter.size());
    
//The centers of reachable spheres are stored in a map. This data will be utilized in visualizing the spheres in the visualizer. 
//TODO there are several maps are implemented. We can get rid of few maps and run large loops. The complexity of accessing map is Olog(n)
     vector<vector<double> > capability_data;
     map<vector<double>, double>  sphereColor;
    vector<vector<double> > poseReach;
    for (multimap<vector<double>, vector<double> >::iterator it = PoseColFilter.begin();it != PoseColFilter.end();++it){
	//Reachability Index D=R/N*100;
        
	float d=float(PoseColFilter.count(it->first))/(PoseCol.size()/newData.size())*100;
	sphereColor.insert(pair<vector<double>, double >(it->first,double(d)));
        poseReach.push_back(it->second);
	}

    ROS_INFO("No of spheres reachable: %lu",sphereColor.size());

//Starting capability map

//Swap the map index and data, as we need to take only the spheres that have reachability index less than 5 for cones
    
    ROS_INFO("All the outer spheres are checked for optimal pose and optimal openning angles for cone representation. May take some time.");
    map<vector<double>, double>  coneSpheres;
    for (map<vector<double>, double> ::iterator it = sphereColor.begin();it != sphereColor.end();++it){
        if(it->second <=20){
	
	coneSpheres.insert(pair<vector<double>, double >(it->first,it->second));
    }
}	
    map<vector<double>, double>  onlySpheres;
    for (map<vector<double>, double> ::iterator it = sphereColor.begin();it != sphereColor.end();++it){
        if(it->second >20){
	
	onlySpheres.insert(pair<vector<double>, double >(it->first,it->second));
    }
}	

    
    for (map<vector<double>, double> ::iterator it = coneSpheres.begin();it != coneSpheres.end();++it)
    {
	geometry_msgs::Pose optiPose;
	point3d sphereCenter;
	sd.convertVectorToPoint(it->first,sphereCenter);
	vector<geometry_msgs::Pose> reachPoseofSphere;
	vector<point3d> reachPoints;
	multimap<vector<double>, vector<double> >::iterator it1;
	for(it1 = PoseColFilter.lower_bound(it->first); it1 !=PoseColFilter.upper_bound(it->first); ++it1){
		geometry_msgs::Pose pp;
		point3d posPoint;
		sd.convertVectorToPose(it1->second,pp);
		sd.poseToPoint(pp,posPoint);
		reachPoseofSphere.push_back(pp);
		reachPoints.push_back(posPoint);
	}
        optiPose=sd.findOptimalPose(reachPoseofSphere, sphereCenter);
        double SFE=0.0;
	map<double, double>  angleSFE;
        for(double angle = 2; angle <=10.0 ; angle +=0.5){

	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	    sd.createConeCloud(optiPose,angle,0.5,cloud);
	    double r_poses=0.0; //Pose that are reachable but not in cone
	    for(int j=0;j<reachPoints.size();j++){
                if(!sd.isPointInCloud(cloud, reachPoints[j]))
		    r_poses +=1;
		
	           }
	    double R_poses = reachPoints.size(); //Total number of filtered pose in that sphere
	
	    multimap<vector<double>, vector<double> >::iterator it2;
	    vector<point3d> reachPointsSphere;
	    for(it2 = PoseCol2.lower_bound(it->first); it2 !=PoseCol2.upper_bound(it->first); ++it2){
	
	        geometry_msgs::Pose pp;
	        point3d posPoint;
	        sd.convertVectorToPose(it2->second,pp);  
	        sd.poseToPoint(pp,posPoint);
	        reachPointsSphere.push_back(posPoint);
		   }
	    double v_poses=0.0;//poses that are in the cone but not reachable
	    for(int k=0;k<reachPointsSphere.size();k++){
	        if(sd.isPointInCloud(cloud, reachPointsSphere[k]))
		  if(std::count(reachPoints.begin(),reachPoints.end(),reachPointsSphere[k])==0)
		       v_poses +=1;
		   }
	     
	     SFE=(r_poses+v_poses)/R_poses;
	     angleSFE.insert(pair<double, double >(SFE, angle));
	     }
            
	    //angleDegrees.push_back(angleSFE.begin()->second);
	    vector<double> capability;
	    capability.push_back(1.0);
            capability.push_back(it->second);
	    capability.push_back(optiPose.position.x);
	    capability.push_back(optiPose.position.y);
	    capability.push_back(optiPose.position.z);
	    capability.push_back(optiPose.orientation.x);
	    capability.push_back(optiPose.orientation.y);
	    capability.push_back(optiPose.orientation.z);
	    capability.push_back(optiPose.orientation.w);
	    capability.push_back(angleSFE.begin()->second);
	    

	    capability_data.push_back(capability);

}



for (map<vector<double>, double> ::iterator it = onlySpheres.begin();it != onlySpheres.end();++it)
    {
          vector<double> capability_sp;
	  capability_sp.push_back(2.0);
          capability_sp.push_back(it->second);
	  capability_sp.push_back(it->first[0]);
	  capability_sp.push_back(it->first[1]);
	  capability_sp.push_back(it->first[2]);
	  capability_sp.push_back(0.0);
	  capability_sp.push_back(0.0);
	  capability_sp.push_back(0.0);
	  capability_sp.push_back(1.0);
	  capability_sp.push_back(0.0);
	  
	  capability_data.push_back(capability_sp);
}

     ROS_INFO("Capability map is created, saving data to database.");


//Starting database

    string path(ros::package::getPath("reuleaux")+"/maps/");
    if (stat(path.c_str(),&st)!=0)
	ROS_INFO("Path does not exist. Creating folder for maps");
	const int dir_err = mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        if (1 == dir_err)
	{
    		ROS_INFO("Error creating directory");
    		exit(1);
	}

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
    filename=string(k.getRobotName())+"_"+boost::lexical_cast<std::string>(Hour)+":"+boost::lexical_cast<std::string>(Min)+"_"+boost::lexical_cast<std::string>(Month)+":"+boost::lexical_cast<std::string>(Day)+":"+boost::lexical_cast<std::string>(Year)+"_"+"r"+str( boost::format("%d") % resolution)+"_"+"sd"+"_"+"rot"+"_"+"capability"+"."+"h5";

    const char * filepath = path.c_str();
    const char * name = filename.c_str();
    char fullpath[100];
    strcpy(fullpath,filepath);
    strcat(fullpath,name);
    ROS_INFO("Saving map %s",filename.c_str());
    hid_t file, group_capability;
    file = H5Fcreate(fullpath, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    group_capability = H5Gcreate(file,"/Capability",H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    hid_t capability_dataset, capability_dataspace;
    const int SX = capability_data.size();                     
    const int SY = 10;
    hsize_t dims2[2];               // dataset dimensions
    dims2[0] = SX;
    dims2[1] = SY;
    double dset2_data[SX][SY];
    for(int i=0;i<capability_data.size();i++){
	for(int j=0;j<capability_data[i].size();j++){
		dset2_data[i][j] = capability_data[i][j];
	}
    }
    capability_dataspace = H5Screate_simple(2, dims2, NULL);
    capability_dataset = H5Dcreate2(group_capability,"capability_dataset",H5T_NATIVE_DOUBLE, capability_dataspace,  H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
    
    H5Dwrite(capability_dataset, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, 
                     dset2_data);

//Closing all
    H5Sclose(capability_dataspace);
    H5Dclose(capability_dataset);
    H5Gclose(group_capability);
    H5Fclose(file);
    time (&finish);
    double dif = difftime (finish,startit);
    ROS_INFO ("Elasped time is %.2lf seconds.", dif );
    ROS_INFO ("Completed");
    ros::spinOnce();
    //sleep(20000);
    return -1;
    loop_rate.sleep();
    count;
  }
  
  return 0;
}

