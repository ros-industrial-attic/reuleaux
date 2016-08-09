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

bool isIkSuccesswithTransformedBase(const geometry_msgs::Pose base_pose, const geometry_msgs::Pose grasp_pose, int& numOfSolns)
{
//Creating a transformation out of base pose
    tf2::Vector3 base_vec(base_pose.position.x, base_pose.position.y, base_pose.position.z);
    tf2::Quaternion base_quat(base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w);
    base_quat.normalize();
    tf2::Transform base_trns;
    base_trns.setOrigin(base_vec);
    base_trns.setRotation(base_quat);

//Inverse of the transformation
    tf2::Transform base_trns_inv;
    base_trns_inv = base_trns.inverse();

//Creating a transformation of grasp pose
    tf2::Vector3 grasp_vec(grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);
    tf2::Quaternion grasp_quat(grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w);
    grasp_quat.normalize();
    tf2::Transform grasp_trns;
    grasp_trns.setOrigin(grasp_vec);
    grasp_trns.setRotation(grasp_quat);

//Transforming grasp pose to origin from where we can check for Ik
    tf2::Transform new_grasp_trns;
    //new_grasp_trns = grasp_trns * base_trns_inv;
    new_grasp_trns =  base_trns_inv * grasp_trns;
//Creating a new grasp pose in the origin co-ordinate
    vector<double> new_grasp_pos;
    tf2::Vector3 new_grasp_vec;
    tf2::Quaternion new_grasp_quat;
    new_grasp_vec =  new_grasp_trns.getOrigin();
    new_grasp_quat =  new_grasp_trns.getRotation();
    new_grasp_quat.normalize();
    new_grasp_pos.push_back(new_grasp_vec[0]);
    new_grasp_pos.push_back(new_grasp_vec[1]);
    new_grasp_pos.push_back(new_grasp_vec[2]);
    new_grasp_pos.push_back(new_grasp_quat[0]);
    new_grasp_pos.push_back(new_grasp_quat[1]);
    new_grasp_pos.push_back(new_grasp_quat[2]);
    new_grasp_pos.push_back(new_grasp_quat[3]);

//Check the new grasp_pose for Ik
    Kinematics k;
    std::vector<double> joints;
    
    joints.resize(6);
    if(k.isIKSuccess(new_grasp_pos, joints, numOfSolns))
      return true;
    else
      return false;

} 

	

int main(int argc, char **argv)
{
  if (argc<2){
	ROS_ERROR_STREAM("Please provide the name of the reachability map. If you have not created it yet, Please create the map by running the create reachability map node in map_creator package");
	return 1;
    } 
  else{
  ros::init(argc, argv, "inverse_workspace");
  ros::NodeHandle n;
  ros::Publisher arrow_pub = n.advertise<geometry_msgs::PoseArray>("orientations", 1);
  ros::Publisher arrow_pub2 = n.advertise<geometry_msgs::PoseArray>("orientations2", 1);
  time_t startit,finish;
  time (&startit);
  ros::Rate loop_rate(10);
  ros::Rate poll_rate(100);
  while(arrow_pub.getNumSubscribers() == 0)
    poll_rate.sleep();

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

    multiset<pair<double, vector<double> > > scoreWithSp;
    for (map<vector<double>, double > ::iterator it = SphereCol.begin();it != SphereCol.end();++it)
    {
      scoreWithSp.insert(pair<double, vector<double> >(it->second, it->first ));
    }

    ROS_INFO("Numer of Spheres : %lu",scoreWithSp.size());
    
    vector<vector<double> > highScoreSp;
    for (multiset<pair<double, vector<double> > > ::reverse_iterator it = scoreWithSp.rbegin();it != scoreWithSp.rend();++it)
    {
	 highScoreSp.push_back(it->second);

    }

    ROS_INFO("Size of high score sp: %lu", highScoreSp.size()); //filtered sp centers based on their scores
    vector<geometry_msgs::Pose> probBasePoses;

   int numofSp = 10;//From how many spheres we are collecting all the poses
    for(int i=0;i<numofSp;++i) 
    {
      multimap<vector<double>, vector<double> >::iterator it;
      for(it = PoseColFilter.lower_bound(highScoreSp[i]); it !=PoseColFilter.upper_bound(highScoreSp[i]); ++it){
	geometry_msgs::Pose pp;
        sd.convertVectorToPose(it->second,pp); 
        probBasePoses.push_back(pp);

      }
    }

    cout<<"Size of Probable Base poses: "<<probBasePoses.size()<<" with "<<numofSp<<"Sphere"<<endl;

    multiset<pair<int, vector<double> > >basePoseWithHits;
    
    for(int i=0;i<probBasePoses.size();++i)
    {
      int numofHits = 0;
      for(int j=0;j<grasp_poses.size();++j)
      {
        int nsolns = 0;
	numofHits+=isIkSuccesswithTransformedBase(probBasePoses[i],grasp_poses[j], nsolns);
        
      }
	//cout<<numofHits<<endl;
	vector<double> baseP;
        sd.convertPoseToVector(probBasePoses[i], baseP);
	basePoseWithHits.insert(pair<int, vector<double> >(numofHits , baseP));
        //scoreWithSp.insert(pair<double, vector<double> >(it->second, it->first ));
    }
    


    vector<geometry_msgs::Pose> final_base_poses;
    for (multiset<pair<int, vector<double> > > ::iterator it = basePoseWithHits.begin();it != basePoseWithHits.end();++it)
    {
	//cout<<"Pose: "<<it->second[0]<<it->second[1]<<it->second[2]<<"=> has "<<it->first<<" solutions." <<endl;
	if((it->first)>=grasp_poses.size())
	{
	  geometry_msgs::Pose final_base;
	  sd.convertVectorToPose(it->second, final_base);
          final_base_poses.push_back(final_base);
	}

    }

    cout<<"# of final base poses: "<<final_base_poses.size()<<endl;

 int numofFinalPose = 10;//How many optimal base pose we need 
 geometry_msgs::PoseArray po;  
 po.header.stamp=ros::Time::now();
    po.header.frame_id="/base_link"; 
cout<<"Checking for ikReachability of grasp poses from new base poses"<<endl; 
for(int i=0;i<numofFinalPose;++i)
    {
    
    cout<<"Optimal BASE POse: "<<final_base_poses[i].position.x<<", "<<final_base_poses[i].position.y<<" , "<<final_base_poses[i].position.z<<"....."<<final_base_poses[i].orientation.x<<", "<<final_base_poses[i].orientation.y<<", "<<final_base_poses[i].orientation.z<<", "<<final_base_poses[i].orientation.w<<endl;
	po.poses.push_back(final_base_poses[i]);
    for(int j=0;j<grasp_poses.size();++j)
    {
      int solns = 0;
      cout<<"Grasp Pose: "<<j<<" is reachable: "<<isIkSuccesswithTransformedBase(final_base_poses[i],grasp_poses[j], solns)<<endl;
     }
   }

arrow_pub.publish(po);

geometry_msgs::PoseArray po2;  
 po2.header.stamp=ros::Time::now();
    po2.header.frame_id="/base_link";   
    for(int i=0;i<grasp_poses.size();++i)
    {
	po2.poses.push_back(grasp_poses[i]);
     }
arrow_pub2.publish(po2);




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
