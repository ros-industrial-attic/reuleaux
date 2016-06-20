#include<ros/ros.h>
#include<time.h>
#include<vector>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include "reuleaux/WorkSpace.h"
#include "geometry_msgs/PoseArray.h"

class WorkSpaceViz
{
public:
    WorkSpaceViz();
    void PointCallback(const reuleaux::WorkSpace::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber work_space_sub;
    ros::Publisher arrow_pub;
    ros::Publisher point_pub;
};

WorkSpaceViz::WorkSpaceViz(){
    arrow_pub=nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",1);
    point_pub=nh_.advertise<visualization_msgs::Marker>("visualization_marker",1);
    work_space_sub=nh_.subscribe<reuleaux::WorkSpace>("workspace",1,&WorkSpaceViz::PointCallback, this);
}
    
void WorkSpaceViz::PointCallback(const reuleaux::WorkSpace::ConstPtr& msg)
{
   
    //ROS_INFO("The len of msg [%i]",msg->point_len);
    
          visualization_msgs::MarkerArray markerArr;
          visualization_msgs::Marker marker;
    
    for (uint32_t i=0;i<msg->poses.size();++i)
    
      {   
          visualization_msgs::Marker marker;
          marker.header.frame_id="/base_link";
    	  marker.header.stamp=ros::Time::now();
   	  marker.ns="points";
    	  
    	  marker.action=visualization_msgs::Marker::ADD;
    	  marker.type=visualization_msgs::Marker::ARROW;
    	  
    	  marker.lifetime=ros::Duration();
          marker.scale.x = 0.05;
          marker.scale.y = 0.005;
          marker.scale.z = 0.005;

    	  marker.color.r = 1.0f;
    	  marker.color.g = 0.0f;
    	  marker.color.b = 0.0f;
    	  marker.color.a = 1.0;
	  marker.id=i;
    	  geometry_msgs::Pose p;
	  marker.pose.position.x=msg->poses[i].position.x;
          marker.pose.position.y=msg->poses[i].position.y;
	  marker.pose.position.z=msg->poses[i].position.z;

	  marker.pose.orientation.w=msg->poses[i].orientation.w;
	  marker.pose.orientation.x=msg->poses[i].orientation.x;
	  marker.pose.orientation.y=msg->poses[i].orientation.y;
          marker.pose.orientation.z=msg->poses[i].orientation.z;
	  
	  markerArr.markers.push_back(marker);
	  
	}
     
     arrow_pub.publish(markerArr);

     
    marker.header.frame_id="/base_link";
    marker.header.stamp=ros::Time::now();
    marker.ns="points";
    marker.id=0;
    marker.action=visualization_msgs::Marker::ADD;
    marker.type=visualization_msgs::Marker::SPHERE_LIST;
    marker.pose.orientation.w = 1.0;
    marker.lifetime=ros::Duration();
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
    
    for (uint32_t i=0;i<msg->points.size();++i)
    
      {   

    	  geometry_msgs::Point p;
	  p.x=msg->points[i].x;
          p.y=msg->points[i].y;
          p.z=msg->points[i].z;
          marker.points.push_back(p);
	  //ROS_INFO("received msg [#%i ] [%f] [%f] [%f]",i,msg->points[i].x,msg->points[i].y,msg->points[i].z);
	   ROS_INFO("received msg [#%i ] ",i);
	}
     point_pub.publish(marker);
     	
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arrows");
    ros::NodeHandle n;
    ros::Rate r(1);
    
    WorkSpaceViz workspace;
    ros::spin();
   
}
