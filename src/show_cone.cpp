#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include "reuleaux/capability.h"
#include <eigen_conversions/eigen_msg.h>
using namespace rviz_visual_tools;

class ConeViz
{
public:
    ConeViz();
    void ConeCallback(const reuleaux::capability::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    ros::Subscriber cone_pos;
    ros::Publisher marker_pub;
};

ConeViz::ConeViz(){
    marker_pub=nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker",1);
    cone_pos=nh_.subscribe<reuleaux::capability>("capability",1,&ConeViz::ConeCallback, this);
}
    
void ConeViz::ConeCallback(const reuleaux::capability::ConstPtr& msg)
{
   
    RvizVisualToolsPtr visual_tools_;
    visual_tools_.reset(new RvizVisualTools("base_link", "/visualization_marker_cone"));
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();
    ROS_INFO("Displaying Axis Cone");

    for (uint32_t i=0;i<msg->ws_points.size();++i){	
    Eigen::Affine3d pose;    
    
    geometry_msgs::Pose p;
    p.position.x=msg->ws_points[i].pose.position.x;
    p.position.y=msg->ws_points[i].pose.position.y;
    p.position.z=msg->ws_points[i].pose.position.z;
    p.orientation.x=msg->ws_points[i].pose.orientation.x;
    p.orientation.y=msg->ws_points[i].pose.orientation.y;
    p.orientation.z=msg->ws_points[i].pose.orientation.z;
    p.orientation.w=msg->ws_points[i].pose.orientation.w;
    tf::poseMsgToEigen(p,pose);
    
    pose=pose*Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());
    
    visual_tools_->publishCone(pose, M_PI/msg->ws_points[i].openning_angle, RED, 0.05);}
    visual_tools_->triggerBatchPublish();


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cones");
    ros::NodeHandle n;
    ros::Rate r(1);
    
    ConeViz cone;
    ros::spin();
   
}
