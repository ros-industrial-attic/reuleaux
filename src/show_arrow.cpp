#include<ros/ros.h>
#include<time.h>
#include<vector>
#include<visualization_msgs/MarkerArray.h>
#include "geometry_msgs/PoseArray.h"

class SphereViz
{
public:
    SphereViz();
    void PointCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber point_pos;
    ros::Publisher marker_pub;
};

SphereViz::SphereViz(){
    marker_pub=nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",1);
    point_pos=nh_.subscribe<geometry_msgs::PoseArray>("orientations",1,&SphereViz::PointCallback, this);
}
    
void SphereViz::PointCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
   
    //ROS_INFO("The len of msg [%i]",msg->point_len);
    
          visualization_msgs::MarkerArray markerArr;
          
    
    for (uint32_t i=0;i<msg->poses.size();++i)
    
      {   
          visualization_msgs::Marker marker;
          marker.header.frame_id="/base_link";
    	  marker.header.stamp=ros::Time::now();
   	  marker.ns="points";
    	  
    	  marker.action=visualization_msgs::Marker::ADD;
    	  marker.type=visualization_msgs::Marker::ARROW;
    	  
    	  marker.lifetime=ros::Duration();
          marker.scale.x = 0.1;
          marker.scale.y = 0.01;
          marker.scale.z = 0.01;

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
	  ROS_INFO("received msg [%f] [%f] [%f]",msg->poses[i].position.x,msg->poses[i].position.y,msg->poses[i].position.z);
          //marker.pose.push_back(p);
	  markerArr.markers.push_back(marker);
	  ROS_INFO("received msg [#%i ] ",i);
	}
     
     marker_pub.publish(markerArr);	
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arrows");
    ros::NodeHandle n;
    ros::Rate r(1);
    
    SphereViz sphere;
    ros::spin();
   
}
