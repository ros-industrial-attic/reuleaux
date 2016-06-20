#include<ros/ros.h>
#include "reuleaux/PointArray.h"
#include "geometry_msgs/PoseArray.h"
#include<octomap/octomap.h>
#include<octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <reuleaux/sphere_discretization.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace octomap;
using namespace std;
using namespace octomath;
using namespace sphere_discretization;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "rand_points");
    ros::NodeHandle n;
  
    ros::Publisher arrow_pub = n.advertise<geometry_msgs::PoseArray>("orientations", 1000);

  //ros::Rate loop_rate(10);
  ros::Rate poll_rate(100);
while(arrow_pub.getNumSubscribers() == 0)
    poll_rate.sleep();
  int count = 0;
  while (ros::ok())
  {
      
     float HI=-1.5, LO=1.5;
     unsigned char maxDepth = 16;
     unsigned char minDepth = 0;
    

    SphereDiscretization sd;
    float r=1;
    float resolution =0.5;
    point3d origin=point3d(0,0,0);
    OcTree* tree=sd.generateBoxTree(origin, r, resolution);
    cout<<"Leaf"<<endl;
    std::vector<point3d> newData;
    for (OcTree::leaf_iterator it=tree->begin_leafs(maxDepth), end=tree->end_leafs();it !=end;++it){
	
	newData.push_back(it.getCoordinate());
		
 	}
    
    cout<<"The # of leaf nodes in first tree: "<<tree->getNumLeafNodes()<<endl;
    cout<<"The # of data in the array: "<<newData.size()<<endl;
    cout<<"Hello"<<endl;
    //point3d origin (0.0, 0.0, 0.0);
    float radius=resolution;
    std::vector<geometry_msgs::Pose> PoseCol;
    for (int i=0;i<newData.size();i++){
    
    
    
	for (double phi = 0.; phi < 2*M_PI; phi += M_PI/7.) // Azimuth [0, 2PI]
    {
        for (double theta = 0.; theta < M_PI; theta += M_PI/7.) // Elevation [0, PI]
        {
            geometry_msgs::Pose pp;
            pp.position.x = radius * cos(phi) * sin(theta) + newData[i].x();
            pp.position.y = radius * sin(phi) * sin(theta) + newData[i].y();
            pp.position.z=  radius * cos(theta) + newData[i].z();
            tf2::Quaternion quat;
            quat.setRPY(0,((M_PI/2)+theta), phi);
            pp.orientation.x=quat.x();
            pp.orientation.y=quat.y();
            pp.orientation.z=quat.z();
            pp.orientation.w=quat.w();
            PoseCol.push_back(pp);        
           }
       }   
   }



   	//cout<<"pose"<<PoseCol[6271].position.x<<" "<<PoseCol[6271].position.y<<" "<<PoseCol[6271].position.z<<" "<<PoseCol[6271].orientation.x<<" "<<PoseCol[6271].orientation.y<<" "<<PoseCol[6271].orientation.z<<" "<<PoseCol[6271].orientation.w<<endl;



    geometry_msgs::PoseArray po;
    po.header.stamp=ros::Time::now();
    po.header.frame_id="/base_link";
    cout<<"Total # of poses: "<<PoseCol.size()<<endl;
    for(int i=0;i<PoseCol.size();++i){
      geometry_msgs::Pose p;
      p.position.x=PoseCol[i].position.x;
      p.position.y=PoseCol[i].position.y;
      p.position.z=PoseCol[i].position.z;
      p.orientation.x=PoseCol[i].orientation.x;
      p.orientation.y=PoseCol[i].orientation.y;
      p.orientation.z=PoseCol[i].orientation.z;
      p.orientation.w=PoseCol[i].orientation.w;
      po.poses.push_back(p);
    }
    
    arrow_pub.publish(po);

    ros::spinOnce();
    sleep(10000);
    //loop_rate.sleep();
    ++count;
  }
  
  return 0;
}

