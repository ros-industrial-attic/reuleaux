#include<ros/ros.h>
#include<time.h>
#include<vector>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include "reuleaux/WsColor.h"
#include "reuleaux/PointColor.h"
#include "geometry_msgs/PoseArray.h"

class WorkSpaceViz
{
public:
    WorkSpaceViz();
    void PointCallback(const reuleaux::WsColor::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber work_space_sub;
    ros::Publisher arrow_pub;
    ros::Publisher point_pub;
};

WorkSpaceViz::WorkSpaceViz(){
    arrow_pub=nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_arrows",1);
    point_pub=nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_spheres",1);
    work_space_sub=nh_.subscribe<reuleaux::WsColor>("workspace",1,&WorkSpaceViz::PointCallback, this);
}
    
void WorkSpaceViz::PointCallback(const reuleaux::WsColor::ConstPtr& msg)
{
   
    //ROS_INFO("The len of msg [%i]",msg->point_len);
    
          visualization_msgs::MarkerArray SphereArr;
          visualization_msgs::MarkerArray PoseArr;
    
    for (uint32_t i=0;i<msg->poses.size();++i)
    
      {   
          visualization_msgs::Marker marker;
          marker.header.frame_id="/base_link";
    	  marker.header.stamp=ros::Time::now();
   	  marker.ns="points";
    	  
    	  marker.action=visualization_msgs::Marker::ADD;
    	  marker.type=visualization_msgs::Marker::ARROW;
    	  
    	  marker.lifetime=ros::Duration();
          marker.scale.x = 0.025;
          marker.scale.y = 0.0025;
          marker.scale.z = 0.0025;

    	  marker.color.r = 0.0f;
    	  marker.color.g = 0.0f;
    	  marker.color.b = 0.0f;
    	  marker.color.a = 0.5;
	  marker.id=i;
    	  geometry_msgs::Pose p;
	  marker.pose.position.x=msg->poses[i].position.x;
          marker.pose.position.y=msg->poses[i].position.y;
	  marker.pose.position.z=msg->poses[i].position.z;

	  marker.pose.orientation.w=msg->poses[i].orientation.w;
	  marker.pose.orientation.x=msg->poses[i].orientation.x;
	  marker.pose.orientation.y=msg->poses[i].orientation.y;
          marker.pose.orientation.z=msg->poses[i].orientation.z;
	  //ROS_INFO("received msg [%f] [%f] [%f]",msg->poses[i].position.x,msg->poses[i].position.y,msg->poses[i].position.z);
          //marker.pose.push_back(p);
	  PoseArr.markers.push_back(marker);
	  //ROS_INFO("received msg [#%i ] ",i);
	}
     
     arrow_pub.publish(PoseArr);

     for (uint32_t i=0;i<msg->points.size();++i)
    
      {   
          visualization_msgs::Marker marker;
          marker.header.frame_id="/base_link";
    	  marker.header.stamp=ros::Time::now();
   	  marker.ns="points";
    	  
    	  marker.action=visualization_msgs::Marker::ADD;
    	  marker.type=visualization_msgs::Marker::SPHERE;
    	  
    	  marker.lifetime=ros::Duration();
          marker.scale.x = 0.2;
          marker.scale.y = 0.2;
          marker.scale.z = 0.2;
	  marker.color.a = 0.9;
	  //ROS_INFO("color [%d] ",msg->points[i].col);
	  if (msg->points[i].col>=90){

    	  	marker.color.r = 0.0f;
    	  	marker.color.g = 0.0f;
    	  	marker.color.b = 1.0f;
    	  	
	  }
	  else if (msg->points[i].col<90 && msg->points[i].col>=70){

    	  	marker.color.r = 0.0f;
    	  	marker.color.g = 0.5f;
    	  	marker.color.b = 0.5f;
    	  	
	  }
	  else if (msg->points[i].col<70 && msg->points[i].col>=50){

    	  	marker.color.r = 0.0f;
    	  	marker.color.g = 1.0f;
    	  	marker.color.b = 0.5f;
    	  	
	  }
	  else if (msg->points[i].col<50 && msg->points[i].col>=30){

    	  	marker.color.r = 0.0f;
    	  	marker.color.g = 1.0f;
    	  	marker.color.b = 0.0f;
    	  	
	  }
	  
 	 else if (msg->points[i].col<30 && msg->points[i].col>=20){

    	  	marker.color.r = 1.0f;
    	  	marker.color.g = 1.0f;
    	  	marker.color.b = 0.0f;
    	  	
	  }
	  else if (msg->points[i].col<20 && msg->points[i].col>=5){

    	  	marker.color.r = 0.65f;
    	  	marker.color.g = 0.35f;
    	  	marker.color.b = 0.0f;
    	  	
	  }
	  else{

    	  	marker.color.r = 1.0f;
    	  	marker.color.g = 0.0f;
    	  	marker.color.b = 0.0f;
    	  	
	  }
	  
	  marker.id=i;
    	  geometry_msgs::Pose p;
	  marker.pose.position.x=msg->points[i].x;
          marker.pose.position.y=msg->points[i].y;
	  marker.pose.position.z=msg->points[i].z;

	  marker.pose.orientation.w=1.0;
	  
	  //ROS_INFO("received msg [%f] [%f] [%f]",msg->poses[i].position.x,msg->poses[i].position.y,msg->poses[i].position.z);
          //marker.pose.push_back(p);
	  SphereArr.markers.push_back(marker);
	  ROS_INFO("received msg [#%i ] ",i);
	}
     
     point_pub.publish(SphereArr);
    
     	
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arrows");
    ros::NodeHandle n;
    ros::Rate r(1);
    
    WorkSpaceViz workspace;
    ros::spin();
   
}
