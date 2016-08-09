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

bool areQuaternionClose(tf2::Quaternion q1, tf2::Quaternion q2)
{
     double dot = q1.dot(q2);
     if(dot<0)
	return false;
     else
	return true;
}

tf2::Quaternion inverseSignQuaternion(tf2::Quaternion q)
{
    tf2::Quaternion q_inv(-q[0], -q[1], -q[2], -q[3]);
    return q_inv;
}

void findOptimalPosebyAverage(const vector<geometry_msgs::Pose> probBasePoses,geometry_msgs::Pose& final_base_pose){
//This Function has been borrowed from http://wiki.unity3d.com/index.php/Averaging_Quaternions_and_Vectors

    double totalVecX, totalVecY, totalVecZ;
    double avgVecX, avgVecY, avgVecZ;  
    vector<tf2::Quaternion> quatCol;
    for(int i=0;i<probBasePoses.size();++i)
    {
      totalVecX += probBasePoses[i].position.x;
      totalVecY += probBasePoses[i].position.y;
      totalVecZ += probBasePoses[i].position.z;
      tf2::Quaternion quats(probBasePoses[i].orientation.x, probBasePoses[i].orientation.y, probBasePoses[i].orientation.z, probBasePoses[i].orientation.w);
      quatCol.push_back(quats);
    }
   
    avgVecX = totalVecX/double(probBasePoses.size());
    avgVecY = totalVecY/double(probBasePoses.size());
    avgVecZ = totalVecZ/double(probBasePoses.size());
    
   
    
    double totalQuatX, totalQuatY, totalQuatZ, totalQuatW;
    double avgQuatX, avgQuatY, avgQuatZ, avgQuatW;
    for(int j=0;j<quatCol.size();++j)
    {
      if(!areQuaternionClose(quatCol[0],quatCol[j]))  
      {
	
        tf2::Quaternion quat_new;
        quat_new = inverseSignQuaternion(quatCol[j]);
       
        totalQuatX +=quat_new[0];
        totalQuatY +=quat_new[1];
        totalQuatZ +=quat_new[2];
        totalQuatW +=quat_new[3];
        }
      else
      {
	totalQuatX +=quatCol[j][0];
        totalQuatY +=quatCol[j][1];
        totalQuatZ +=quatCol[j][2];
        totalQuatW +=quatCol[j][3];
      }
    }
    avgQuatX = totalQuatX/double(probBasePoses.size());
    avgQuatY = totalQuatY/double(probBasePoses.size());
    avgQuatZ = totalQuatZ/double(probBasePoses.size());
    avgQuatW = totalQuatW/double(probBasePoses.size());
    tf2::Quaternion final_base_quat(avgQuatX,avgQuatY,avgQuatZ,avgQuatW);
    final_base_quat.normalize();


    final_base_pose.position.x = avgVecX;
    final_base_pose.position.y = avgVecY;
    final_base_pose.position.z = avgVecZ;
    final_base_pose.orientation.x = final_base_quat[0];
    final_base_pose.orientation.y = final_base_quat[1];
    final_base_pose.orientation.z = final_base_quat[2];
    final_base_pose.orientation.w = final_base_quat[3];
 

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

   int numofFinalPose = 10;

    map_creator::WorkSpace ws;
    for(int i=0;i<numofFinalPose;++i) //How many optimal base pose we need (But each pose is from a single sphere)
    {
      map_creator::WsSphere wss;
      wss.point.x = highScoreSp[i][0];
      wss.point.y = highScoreSp[i][1];
      wss.point.z = highScoreSp[i][2];
      vector<double> basePose;
      basePose.push_back(highScoreSp[i][0]);
      basePose.push_back(highScoreSp[i][1]);
      basePose.push_back(highScoreSp[i][2]);
      multimap<vector<double>, vector<double> >::iterator it;
      for(it = PoseColFilter.lower_bound(basePose); it !=PoseColFilter.upper_bound(basePose); ++it){
	geometry_msgs::Pose pp;
        sd.convertVectorToPose(it->second,pp); 
	wss.poses.push_back(pp);
	}
     ws.WsSpheres.push_back(wss);
    }

    std::vector<geometry_msgs::Pose> final_base_poses;
    for(int i=0;i<ws.WsSpheres.size();++i)
    {
	geometry_msgs::Pose final_base_pose;
	//cout<<ws.WsSpheres[i].poses.size()<<endl;
        findOptimalPosebyAverage(ws.WsSpheres[i].poses,final_base_pose);//Calling the averaging function 
	final_base_poses.push_back(final_base_pose);
    }


 geometry_msgs::PoseArray po;  
 po.header.stamp=ros::Time::now();
    po.header.frame_id="/base_link"; 
cout<<"Checking for ikReachability of grasp poses from new base poses"<<endl; 
for(int i=0;i<final_base_poses.size();++i)
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
