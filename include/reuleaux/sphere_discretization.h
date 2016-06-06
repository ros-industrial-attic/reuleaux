#ifndef SPHERE_DISCRETIZATION_H
#define SPHERE_DISCRETIZATION_H
#include<octomap/octomap.h>
#include<octomap/MapCollection.h>
#include<octomap/math/Utils.h>
#include<iostream>
#include<ros/ros.h>
#include "geometry_msgs/Pose.h"
#include<tf2/LinearMath/Quaternion.h>
#include<vector>
using namespace octomap;
using namespace std;
using namespace octomath;


namespace sphere_discretization{

class SphereDiscretization{
public:
    //SphereDiscretization();

    //~SphereDiscretization();

    OcTree* generateSphereTree(point3d origin, float radius, float resolution);

    OcTree* generateSphereTree2(point3d origin, float radius, float resolution);

    OcTree* generateBoxTree(point3d origin, float diameter, float resolution);

    Pointcloud make_sphere_points(point3d origin, double r);
    vector<geometry_msgs::Pose> make_sphere_poses(point3d origin, double r);

    double irand(int min, int max);

    Pointcloud make_sphere_rand(point3d origin, double r, int sample);

    Pointcloud make_sphere_Archimedes(point3d origin, double r, int sample);

    Pointcloud make_sphere_fibonacci_grid(point3d origin, double r, int sample);
    
    double r8_modp(double x, double y);

    void convertPointToVector(point3d point, vector<double> & data);

    void convertPoseToVector(const geometry_msgs::Pose pose, std::vector<double> & data);


    Pointcloud make_sphere_spiral_points(point3d origin, double r, int sample);
    Pointcloud make_long_lat_grid(point3d origin, double r, int sample, int lat_num, int lon_num);
};

    

}// namespace sphere_discretization
#endif  // SPHERE_DISCRETIZATION_H

