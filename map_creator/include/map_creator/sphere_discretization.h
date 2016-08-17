#ifndef SPHERE_DISCRETIZATION_H
#define SPHERE_DISCRETIZATION_H
#include<octomap/octomap.h>
#include<octomap/MapCollection.h>
#include<octomap/math/Utils.h>
#include<iostream>
#include<ros/ros.h>
#include "geometry_msgs/Pose.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

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

    void convertPointToVector(const point3d point, vector<double> & data);
    void convertVectorToPoint(const std::vector<double> data, point3d & point);
    void convertPoseToVector(const geometry_msgs::Pose pose, std::vector<double> & data);
    void convertVectorToPose(const std::vector<double> data, geometry_msgs::Pose & pose);
    geometry_msgs::Pose findOptimalPose(const vector<geometry_msgs::Pose> poses, point3d origin);


    void createConeCloud(const geometry_msgs::Pose pose, const double angle, const double scale,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void poseToPoint(const geometry_msgs::Pose pose, point3d & point);

    bool isPointInCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, point3d point);

    Pointcloud make_sphere_spiral_points(point3d origin, double r, int sample);
    Pointcloud make_long_lat_grid(point3d origin, double r, int sample, int lat_num, int lon_num);

    float distanceL2norm(const point3d p1, const point3d p2);

     
    void poseToEigenVector(const geometry_msgs::Pose pose, Eigen::VectorXd& vec);

    void findOptimalPosebyPCA(const vector<geometry_msgs::Pose> probBasePoses,geometry_msgs::Pose& final_base_pose);

    bool areQuaternionClose(tf2::Quaternion q1, tf2::Quaternion q2);

    tf2::Quaternion inverseSignQuaternion(tf2::Quaternion q);

    void findOptimalPosebyAverage(const vector<geometry_msgs::Pose> probBasePoses,geometry_msgs::Pose& final_base_pose);

    void associatePose(multimap<vector<double>, vector<double> >& baseTrnsCol, const vector<geometry_msgs::Pose> grasp_poses, const multimap<vector<double>, vector<double> > PoseColFilter, const float resolution);  


};

   

}// namespace sphere_discretization
#endif  // SPHERE_DISCRETIZATION_H

