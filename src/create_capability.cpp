

#include<ros/ros.h>
#include <ros/package.h>
#include "reuleaux/capability.h"
#include "reuleaux/WsColor.h"
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <reuleaux/sphere_discretization.h>
#include <reuleaux/kinematics.h>
#include <sys/types.h> 
#include <sys/stat.h> 
#include <ctime>
#include <string>
#include <time.h>



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
    ros::Publisher workspace_pub = n.advertise<reuleaux::capability>("capability", 10);
    ros::Publisher point_pub = n.advertise<reuleaux::WsColor>("workspace", 1);

    //ros::Publisher principal_component = n.advertise<geometry_msgs::Pose>("pc", 1);

   
  
  
  ros::Rate poll_rate(100);
while(workspace_pub.getNumSubscribers() == 0)
    poll_rate.sleep();
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
    float resolution = 0.1;
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
        if(it->second <=10){
	
	coneSpheres.insert(pair<vector<double>, double >(it->first,it->second));
    }
}
    ROS_INFO("Total number of cones: %lu",coneSpheres.size());
    map<vector<double>, double>  onlySpheres;
    for (map<vector<double>, double> ::iterator it = sphereColor.begin();it != sphereColor.end();++it){
        if(it->second >20){
	
	onlySpheres.insert(pair<vector<double>, double >(it->first,it->second));
    }
}	

    ROS_INFO("Total number of sp: %lu",onlySpheres.size());  
    int it_num =0;  
    for (map<vector<double>, double> ::iterator it = coneSpheres.begin();it != coneSpheres.end();++it)
    {
	it_num +=1;
	ROS_INFO("Processing cone no#: %d", it_num); 
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
        for(double angle = 10.0; angle >=2.5 ; angle -=0.5){

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



    reuleaux::capability cap;
    cap.header.stamp=ros::Time::now();
    cap.header.frame_id="/base_link";
    for(int i=0;i<capability_data.size();i++)
    {
	reuleaux::ws_point ws;
	
        
	ws.openning_angle = capability_data[i][9];
	ws.pose.position.x = capability_data[i][2];
	ws.pose.position.y = capability_data[i][3];
	ws.pose.position.z = capability_data[i][4];
	ws.pose.orientation.x = capability_data[i][5];
	ws.pose.orientation.y = capability_data[i][6];
	ws.pose.orientation.z = capability_data[i][7];
	ws.pose.orientation.x = capability_data[i][8];
	cap.ws_points.push_back(ws);
    }
   

    workspace_pub.publish(cap);
    reuleaux::WsColor ws;
    ws.header.stamp=ros::Time::now();
    ws.header.frame_id="/base_link";
   for (map<vector<double>, double> ::iterator it = onlySpheres.begin();it != onlySpheres.end();++it){

    reuleaux::PointColor p;
    p.x=it->first[0];
      p.y=it->first[1];
      p.z=it->first[2];
      p.col=it->second;
      ws.points.push_back(p);
    }

   point_pub.publish(ws);

    time (&finish);
    double dif = difftime (finish,startit);
    ROS_INFO ("Elasped time is %.2lf seconds.", dif );
    ROS_INFO ("Completed");
    ros::spinOnce();
    sleep(20000);
    //loop_rate.sleep();
    ++count;
  }
  
  return 0;
}
