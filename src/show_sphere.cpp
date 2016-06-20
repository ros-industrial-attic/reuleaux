#include<ros/ros.h>
#include<time.h>
#include<vector>
#include<visualization_msgs/Marker.h>
#include "reuleaux/PointArray.h"

class SphereViz
{
public:
    SphereViz();
    void PointCallback(const reuleaux::PointArray::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber point_pos;
    ros::Publisher marker_pub;
};

SphereViz::SphereViz(){
    marker_pub=nh_.advertise<visualization_msgs::Marker>("visualization_marker",1);
    point_pos=nh_.subscribe<reuleaux::PointArray>("points",1,&SphereViz::PointCallback, this);
}
    
void SphereViz::PointCallback(const reuleaux::PointArray::ConstPtr& msg)
{
   
    //ROS_INFO("The len of msg [%i]",msg->point_len);
    
    visualization_msgs::Marker marker;
    marker.header.frame_id="/base_link";
    marker.header.stamp=ros::Time::now();
    marker.ns="points";
    marker.id=0;
    marker.action=visualization_msgs::Marker::ADD;
    marker.type=visualization_msgs::Marker::SPHERE_LIST;
    marker.pose.orientation.w = 1.0;
    marker.lifetime=ros::Duration();
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    
    
    
    
    for (uint32_t i=0;i<msg->points.size();++i)
    
      {
    	  geometry_msgs::Point p;
	  p.x=msg->points[i].x;
          p.y=msg->points[i].y;
          p.z=msg->points[i].z;
          marker.points.push_back(p);
	  ROS_INFO("received msg [#%i ] [%f] [%f] [%f]",i,msg->points[i].x,msg->points[i].y,msg->points[i].z);
	}
     marker_pub.publish(marker);	
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spheres");
    ros::NodeHandle n;
    ros::Rate r(1);
    
    SphereViz sphere;
    ros::spin();
   
}
