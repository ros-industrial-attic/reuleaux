#include <ros/ros.h>
#include "map_creator/capability.h"
#include <map_creator/hdf5_dataset.h>

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    ROS_ERROR_STREAM("Please provide the name of the Capability map. If you have not created it yet, Please create the "
                     "map by running the create cpability map launch file in map_creator");
    return 1;
  }
  else
  {
    ros::init(argc, argv, "workspace");
    ros::NodeHandle n;
    bool latchOn = 1;
    ros::Publisher workspace_pub = n.advertise< map_creator::capability >("capability_map", 1);

    ros::Rate loop_rate(10);
    int count = 0;

    hdf5_dataset::Hdf5Dataset h5(argv[1]);
    h5.open_cap();

    std::vector<std::vector<double> > capData;
    float resolution;
    h5.loadCapMapFromDataset(capData, resolution);

    // Creating messages

    map_creator::capability cp;
    cp.header.stamp = ros::Time::now();
    cp.header.frame_id = "/base_link";
    cp.resolution = resolution;

    for(std::vector<std::vector<double> >::iterator it = capData.begin(); it != capData.end(); ++it)
    {
      map_creator::capShape cpSp;
      cpSp.identifier = (*it)[0];
      cpSp.ri = (*it)[1];
      cpSp.angleSFE = (*it)[2];
      cpSp.pose.position.x = (*it)[3];
      cpSp.pose.position.y = (*it)[4];
      cpSp.pose.position.z = (*it)[5];
      cpSp.pose.orientation.x = (*it)[6];
      cpSp.pose.orientation.x = (*it)[7];
      cpSp.pose.orientation.x = (*it)[8];
      cpSp.pose.orientation.x = (*it)[9];
      cp.capShapes.push_back(cpSp);

    }
    while (ros::ok())
    {
      workspace_pub.publish(cp);
      ros::spinOnce();
      sleep(1);
      ++count;
    }
  }
  return 0;
}
