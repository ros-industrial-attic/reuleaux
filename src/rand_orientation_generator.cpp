#include<ros/ros.h>
#include "geometry_msgs/PoseArray.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rand_orientations");
    ros::NodeHandle n;
  
    ros::Publisher orient_pub = n.advertise<geometry_msgs::PoseArray>("orientations", 1);

  //ros::Rate loop_rate(10);
  ros::Rate poll_rate(100);
while(orient_pub.getNumSubscribers() == 0)
    poll_rate.sleep();
  int count = 0;
  while (ros::ok())
  {
    
      float HI=-1.5, LO=1.5;
	      
	      
      geometry_msgs::PoseArray po;
      po.header.stamp=ros::Time::now();
      po.header.frame_id="/base_link";
      
      for (uint32_t i=0;i<5000;++i)
      {
	      geometry_msgs::Pose p;
	      

	      float r1 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
	      float r2 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
	      float r3 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
	      p.position.x=r1;
	      p.position.y=r2;
	      p.position.z=r3;
	      p.orientation.x=r1;
	      p.orientation.y=r2;
	      p.orientation.z=r3;
	      p.orientation.w=1.0;
              ROS_INFO("I send: [%f] [%f] [%f]", r1,r2,r3);
      po.poses.push_back(p);
      
    
    }

    
    orient_pub.publish(po);

    ros::spinOnce();
    sleep(100000);
    //loop_rate.sleep();
    ++count;
  }
  
  return 0;
}

