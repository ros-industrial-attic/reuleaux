#include<ros/ros.h>
#include "reuleaux/PointArray.h"
#include "geometry_msgs/Point.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rand_points");
    ros::NodeHandle n;
  
    ros::Publisher point_pub = n.advertise<reuleaux::PointArray>("points", 1000);

  //ros::Rate loop_rate(10);
  ros::Rate poll_rate(100);
while(point_pub.getNumSubscribers() == 0)
    poll_rate.sleep();
  int count = 0;
  while (ros::ok())
  {
    
      float HI=-1.5, LO=1.5;
	      
	      
      reuleaux::PointArray po;
      //po.point_len=5000;
      for (uint32_t i=0;i<5000;++i)
      {
	      geometry_msgs::Point32 p;
	      float r1 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
	      float r2 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
	      float r3 = LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)));
	      p.x=r1;
	      p.y=r2;
	      p.z=r3;
              ROS_INFO("I send: [%f] [%f] [%f]", r1,r2,r3);
      po.points.push_back(p);
    
    }

    
    point_pub.publish(po);

    ros::spinOnce();
    sleep(100000);
    //loop_rate.sleep();
    ++count;
  }
  
  return 0;
}

