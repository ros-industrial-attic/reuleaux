#include<ros/ros.h>
#include "reuleaux/WsColor.h"
#include<octomap/octomap.h>
#include<octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <reuleaux/sphere_discretization.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseArray.h"
#include<map>
#include<reuleaux/kinematics.h>

using namespace octomap;
using namespace std;
using namespace octomath;
using namespace sphere_discretization;
using namespace kinematics;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "workspace");
    ros::NodeHandle n;
  
    

    ros::Publisher workspace_pub = n.advertise<reuleaux::WsColor>("workspace", 1000);
  //ros::Rate loop_rate(10);
  ros::Rate poll_rate(100);
while(workspace_pub.getNumSubscribers() == 0)
    poll_rate.sleep();
  int count = 0;
  while (ros::ok())
  {
      
     float HI=-1.5, LO=1.5;
     unsigned char maxDepth = 16;
     unsigned char minDepth = 0;
    

    SphereDiscretization sd;
    float r=1;
    float resolution =0.1;
    point3d origin=point3d(0,0,0);
    OcTree* tree=sd.generateBoxTree(origin, r, resolution);
    cout<<"Leaf"<<endl;
    std::vector<point3d> newData;
    cout<<"Creating the box and discretizing with resolution: "<<resolution<<endl;
    for (OcTree::leaf_iterator it=tree->begin_leafs(maxDepth), end=tree->end_leafs();it !=end;++it){
	
	newData.push_back(it.getCoordinate());
		
 	}
    //cout<<"The # of leaf nodes in first tree: "<<tree->getNumLeafNodes()<<endl;
    cout<<"Total no of spheres now: "<<newData.size()<<endl;
    cout<<"Please hold ON. Spheres are discretized and all of the poses are checked for Ik solutions. May take some time"<<endl;



    float radius=resolution;
    vector<geometry_msgs::PoseArray>  pose_Col;

    multimap<vector<double>, vector<double> > PoseCol;
    for (int i=0;i<newData.size();i++){
	
	vector<geometry_msgs::Pose> pose;
	vector<double> sphere_coord;
	sd.convertPointToVector(newData[i],sphere_coord);
        
	pose=sd.make_sphere_poses(newData[i],radius);
	for(int j=0;j<pose.size();j++){
		
		vector<double> point_on_sphere;
   		sd.convertPoseToVector(pose[j],point_on_sphere);
        
   	//cout<<"Data from vector"<<point_on_sphere[0]<<" "<<point_on_sphere[1]<<" "<<point_on_sphere[2]<<" "<<point_on_sphere[3]<<" "<<point_on_sphere[4]<<" "<<point_on_sphere[5]<<" "<<point_on_sphere[6]<<endl;
	PoseCol.insert(pair<vector<double>, vector<double> >(point_on_sphere,sphere_coord));
	}
}

Kinematics k;
    multimap<vector<double>, vector<double> > PoseColFilter;
    vector<vector<double> > ikSolutions;
    for (multimap<vector<double>, vector<double> >::iterator it = PoseCol.begin();it != PoseCol.end();++it){
        std::vector<double> joints;
	joints.resize(6);
	if (k.isIKSuccess(it->first,joints)){
		PoseColFilter.insert(pair<vector<double>, vector<double> >(it->second,it->first));
	        ikSolutions.push_back(joints);
		//cout<<it->first[0]<<" "<<it->first[1]<<" "<<it->first[2]<<" "<<it->first[3]<<" "<<it->first[4]<<" "<<it->first[5]<<" "<<it->first[6]<<endl;
	    }
    }
cout<<"Total number of poses: "<<PoseCol.size()<<endl;
cout<<"Total number of reachable poses: "<<PoseColFilter.size()<<endl;
   
    reuleaux::WsColor ws;

    
    ws.header.stamp=ros::Time::now();
    ws.header.frame_id="/base_link";
    
    for (multimap<vector<double>, vector<double> >::iterator it = PoseColFilter.begin();it != PoseColFilter.end();++it){

      geometry_msgs::Pose p;
      p.position.x=it->second[0];
      p.position.y=it->second[1];
      p.position.z=it->second[2];
      p.orientation.x=it->second[3];
      p.orientation.y=it->second[4];
      p.orientation.z=it->second[5];
      p.orientation.w=it->second[6];
      ws.poses.push_back(p);
    }

map<vector<double>, int>  sphereColor;
for (multimap<vector<double>, vector<double> >::iterator it = PoseColFilter.begin();it != PoseColFilter.end();++it){
	//Reachability Index D=R/N*100;
	float d=float(PoseColFilter.count(it->first))/(PoseCol.size()/newData.size())*100;
	cout<<d<<endl;
	sphereColor.insert(pair<vector<double>, int >(it->first,PoseColFilter.count(it->first)));
	
}

cout<<"No of spheres reachable: "<<sphereColor.size()<<endl;
	
   for (map<vector<double>, int> ::iterator it = sphereColor.begin();it != sphereColor.end();++it){
      reuleaux::PointColor p;
      p.x=it->first[0];
      p.y=it->first[1];
      p.z=it->first[2];
      p.col=it->second;
      ws.points.push_back(p);
    }
	
   workspace_pub.publish(ws);

    ros::spinOnce();
    sleep(2);
    //loop_rate.sleep();
    ++count;
}
return 0;
}

