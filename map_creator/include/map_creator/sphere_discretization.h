#ifndef SPHERE_DISCRETIZATION_H
#define SPHERE_DISCRETIZATION_H
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <iostream>
#include <ros/ros.h>
#include "geometry_msgs/Pose.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <vector>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/surface/convex_hull.h>
#include <pcl_ros/transforms.h>
//#include <pcl/segmentation/extract_polygonal_prism_data.h>

namespace sphere_discretization
{
class SphereDiscretization
{
public:
   SphereDiscretization(){}

  ~SphereDiscretization(){}

  //! Generating a sphere octree by insertRay
  octomap::OcTree* generateSphereTree(const octomap::point3d& origin, float radius, float resolution);

  //! Generating a sphere octree by creating poincloud and pushing to octree
  octomap::OcTree* generateSphereTree2(const octomap::point3d& origin, float radius, float resolution);

  //! Generating a box octree
  octomap::OcTree* generateBoxTree(const octomap::point3d& origin, float diameter, float resolution);

  //! Creating a sphere with points and return poincloud
  octomap::Pointcloud make_sphere_points(const octomap::point3d& origin, double r);

  //! Creating a sphere and and create poses on the outer sphere and return vector of poses
  void make_sphere_poses(const octomap::point3d &origin, double r, std::vector< geometry_msgs::Pose >& pose_Col);

  //! Creating random doubles
  double irand(int min, int max);

  //! Creating sphere with random points
  octomap::Pointcloud make_sphere_rand(octomap::point3d origin, double r, int sample);

  //! Creating sphere by Archimedes theorem
  octomap::Pointcloud make_sphere_Archimedes(octomap::point3d origin, double r, int sample);

  //! Creating sphere by Fibonacci grid
  octomap::Pointcloud make_sphere_fibonacci_grid(octomap::point3d origin, double r, int sample);

  //! Creates sphere spiral points and returns pointcloud
  octomap::Pointcloud make_sphere_spiral_points(const octomap::point3d &origin, double r, int sample);

  //! Creates long lat grid on a sphere
  octomap::Pointcloud make_long_lat_grid(const octomap::point3d &origin, double r, int sample, int lat_num, int lon_num);

  //! Returns non-negative numbers of R8 division
  double r8_modp(double x, double y);

  //! Convering vector[3] to point
  void convertPointToVector(const octomap::point3d point, std::vector< double >& data);

  //! Converting point to vector[3]
  void convertVectorToPoint(const std::vector< double > data, octomap::point3d& point);

  //! Converting geometry_msgs::Pose to vector[7]
  void convertPoseToVector(const geometry_msgs::Pose &pose, std::vector< double >& data);

  //! Converting vector[7] to geometry_msgs::Pose
  void convertVectorToPose(const std::vector< double >& data, geometry_msgs::Pose& pose);

  //! Finding optimal pose by averaging all and returning pose
  geometry_msgs::Pose findOptimalPose(const std::vector< geometry_msgs::Pose >& poses, const octomap::point3d& origin);

  //! Creates cone by PointCloud
  void createConeCloud(const geometry_msgs::Pose &pose, const double angle, const double scale,
					   pcl::PointCloud< pcl::PointXYZ >::Ptr cloud);

  //! Creates point by removing orientation part of a pose
  void poseToPoint(const geometry_msgs::Pose& pose, octomap::point3d& point);

  //! Defines if a point belongs to a pointcloud. First crates a hull of the poincloud and computes Area, then adds the
  //point to the pointcloud and again computes new Area. If the New Area is bigger than the previous area, the point is
  //outside poincloud, otherwise inside
  bool isPointInCloud(pcl::PointCloud< pcl::PointXYZ >::Ptr cloud, octomap::point3d point);

  //! Finds L2 norm distance between two points
  float distanceL2norm(const octomap::point3d& p1, const octomap::point3d& p2);

  //! Converts geometry_msgs::Pose to Eigen Vector
  void poseToEigenVector(const geometry_msgs::Pose& pose, Eigen::VectorXd& vec);

  //! Finds optimal pose of given poses by Principal Component Optimization
  void findOptimalPosebyPCA(const std::vector< geometry_msgs::Pose >& probBasePoses, geometry_msgs::Pose& final_base_pose);

  //! Finds if two quaternions are close
  bool areQuaternionClose(const tf2::Quaternion &q1, const tf2::Quaternion &q2);

  //! Computes the inverse signature of a quaternion
  tf2::Quaternion inverseSignQuaternion(const tf2::Quaternion& q);

  //! Finds optimal pose of given poses by average
  void findOptimalPosebyAverage(const std::vector< geometry_msgs::Pose > probBasePoses,
                                geometry_msgs::Pose& final_base_pose);

  //! Given grasp poses and multimap structure of an inverse reachability map, transforms every pose of the ir map with
  //grasp poses, and calculates nearest neighbor search to associate poses with belonging spheres.
  void associatePose(std::multimap<std::vector<double>, std::vector<double> > &baseTrnsCol,
					 const std::vector< geometry_msgs::Pose >& grasp_poses,
					 const std::multimap<std::vector<double>, std::vector<double> > &PoseColFilter, const float resolution);

  //! Compare two vectors, of length 3, for multimap search
  struct vec_comp_
  {
    bool operator()(const std::vector< float >& v1, const std::vector< float >& v2) const
    {
      // TODO: need to add tolerance as a function of the map resolution; resolution maybe needs to be a class variable
      // but this appears to work fine for now
      float tol = 0.001;
      return (fabs(v1[0] - v2[0]) < tol) && (fabs(v1[1] - v2[1]) < tol) && (fabs(v1[2] - v2[2]) < tol);
    }
  };
};

}  // namespace sphere_discretization
#endif  // SPHERE_DISCRETIZATION_H
