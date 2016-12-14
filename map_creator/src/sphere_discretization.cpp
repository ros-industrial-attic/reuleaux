#include <map_creator/sphere_discretization.h>

namespace sphere_discretization
{
// SphereDiscretization::SphereDiscretization(){}

octomap::OcTree* SphereDiscretization::generateSphereTree(const octomap::point3d& origin, float radius, float resolution)
{
  octomap::OcTree* tree = new octomap::OcTree(resolution);
  octomap::point3d point_on_surface = origin;

  point_on_surface.x() += radius;
  for (int i = 0; i < 360; i++)
  {
    for (int j = 0; j < 360; j++)
    {
      if (!tree->insertRay(origin, origin + point_on_surface))
      {
        //cout << "Error while inserting ray from " << origin << " to " << point_on_surface << endl;
        ROS_ERROR_STREAM("Error while inserting ray from " << origin << " to " << point_on_surface);
      }
      point_on_surface.rotate_IP(0, 0, DEG2RAD(1.));
    }
    point_on_surface.rotate_IP(0, DEG2RAD(1.0), 0);
  }
  return tree;
}

octomap::OcTree* SphereDiscretization::generateSphereTree2(const octomap::point3d& origin, float radius, float resolution)
{
  octomap::OcTree* tree = new octomap::OcTree(resolution);
  octomap::point3d point_on_surface = origin;
  point_on_surface.x() += radius;
  unsigned sphere_beams = 500;
  double angle = 2.0 * M_PI / double(sphere_beams);

  octomap::Pointcloud p;
  p.reserve(sphere_beams*sphere_beams);

  for (unsigned i = 0; i < sphere_beams; i++)
  {
    for (unsigned j = 0; j < sphere_beams; j++)
    {
      p.push_back(origin + point_on_surface);
      point_on_surface.rotate_IP(0, 0, angle);
    }
    point_on_surface.rotate_IP(0, angle, 0);
  }
  tree->insertPointCloud(p, origin);
  return tree;
}

octomap::OcTree* SphereDiscretization::generateBoxTree(const octomap::point3d& origin, float diameter, float resolution)
{
  octomap::OcTree* tree = new octomap::OcTree(resolution / 2);
  octomap::Pointcloud p;
  for (float x = origin.x() - diameter * 1.5; x <= origin.x() + diameter * 1.5; x += resolution)
  {
    for (float y = origin.y() - diameter * 1.5; y <= origin.y() + diameter * 1.5; y += resolution)
    {
      for (float z = origin.z() - diameter * 1.5; z <= origin.z() + diameter * 1.5; z += resolution)
      {
        // tree ->insertRay(origin, point3d(x,y,z));
        octomap::point3d point;
        point.x() = x;
        point.y() = y;
        point.z() = z;
        tree->updateNode(point, true);
      }
    }
  }

  return tree;
};

octomap::Pointcloud SphereDiscretization::make_sphere_points(const octomap::point3d& origin, double r)
{
  octomap::Pointcloud spherePoints;
  spherePoints.reserve( 7*7*2 );
  for (double phi = 0.; phi < 2 * M_PI; phi += M_PI / 7.)  // Azimuth [0, 2PI]
  {
    for (double theta = 0.; theta < M_PI; theta += M_PI / 7.)  // Elevation [0, PI]
    {
      octomap::point3d point;
      point.x() = r * cos(phi) * sin(theta) + origin.x();
      point.y() = r * sin(phi) * sin(theta) + origin.y();
      point.z() = r * cos(theta) + origin.z();
      spherePoints.push_back(point);
    }
  }
  return spherePoints;
}


void SphereDiscretization::make_sphere_poses(const octomap::point3d& origin, double r, std::vector< geometry_msgs::Pose >& pose_Col)
{
  const double DELTA = M_PI / 5.;
  const unsigned MAX_INDEX = (2 * 5 * 5);
  static std::vector<geometry_msgs::Vector3> position_vector(MAX_INDEX);
  static std::vector<tf2::Quaternion> quaternion(MAX_INDEX);
  static bool initialized = false;

  if( !initialized ){
    initialized=true;
    unsigned index = 0;
    for (double phi = 0; phi < 2*M_PI; phi += DELTA)  // Azimuth [0, 2PI]
    {
      for (double theta = 0; theta < M_PI; theta += DELTA)  // Elevation [0, PI]
      {
        position_vector[index].x = cos(phi) * sin(theta);
        position_vector[index].y = sin(phi) * sin(theta);
        position_vector[index].z = cos(theta);

        tf2::Quaternion quat;
        quat.setRPY(0, ((M_PI / 2) + theta), phi);
        // quat=quat*quat2;
        quat.normalize();
        quaternion[index] = quat;
        index++;
      }
    }
  }
  pose_Col.reserve( MAX_INDEX );
  pose_Col.clear();

  geometry_msgs::Pose pose;
  // TODO Most of the robots have a roll joint as their final joint which can move 0 to 2pi. So if a pose is reachable,
  // then the discretization of roll poses are also reachable. It will increase the data, so we have to decide if we
  // have to use it. The robot whose final roll joint cannot move 0 to 2pi, we have to use it.
  // for (double rot = 0.; rot < 2*M_PI; rot += M_PI/6){

  for (int index = 0; index < MAX_INDEX; index++)
  {
    pose.position.x = r * position_vector[index].x + origin.x();
    pose.position.y = r * position_vector[index].y + origin.y();
    pose.position.z = r * position_vector[index].z + origin.z();
    const tf2::Quaternion& quat = quaternion[index];
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    pose_Col.push_back(pose);
  }
}

double SphereDiscretization::irand(int min, int max)
{
  return ((double)rand() / ((double)RAND_MAX + 1.0)) * (max - min) + min;
}

octomap::Pointcloud SphereDiscretization::make_sphere_rand(octomap::point3d origin, double r, int sample)
{
  octomap::Pointcloud spherePoints;
  spherePoints.reserve(sample);

  double theta = 0, phi = 0;
  for (int i = 0; i < sample; i++)
  {
    octomap::point3d point;
    theta = 2 * M_PI * irand(0, 1);
    phi = acos(2 * irand(0, 1) - 1.0);
    point.x() = float(r * cos(theta) * sin(phi) + origin.x());
    point.y() = float(r * sin(theta) * sin(phi) + origin.y());
    point.z() = float(r * cos(phi) + origin.z());
    spherePoints.push_back(point);
    // cout<<theta<<endl;
  }
  return spherePoints;
}

octomap::Pointcloud SphereDiscretization::make_sphere_Archimedes(octomap::point3d origin, double r, int sample)
{
  octomap::Pointcloud spherePoints;
  spherePoints.reserve(sample*2);

  double theta = 0, phi = 0;
  for (int i = 0; i < sample / 2; i++)
  {
    octomap::point3d point;
    octomap::point3d point2;
    theta = 2 * M_PI * irand(-1, 1);
    point.z() = r * irand(0, 1) + origin.z();
    point.x() = r * sqrt(1 - point.z() * point.z()) * cos(theta) + origin.x();
    point.y() = r * sqrt(1 - point.z() * point.z()) * sin(theta) + origin.y();
    point2.z() = -point.z();
    point2.y() = -point.y();
    point2.x() = -point.x();
    spherePoints.push_back(point);
    spherePoints.push_back(point2);
    //cout << point.x() << " " << point.y() << " " << point.z() << endl;
    ROS_INFO_STREAM(point.x() << " " << point.y() << " " << point.z());
  }
  return spherePoints;
}

octomap::Pointcloud SphereDiscretization::make_sphere_fibonacci_grid(octomap::point3d origin, double r, int sample)
{
  double ng;
  double r_phi = (1.0 + sqrt(5.0)) / 2.0;
  octomap::Pointcloud spherePoints;
  spherePoints.reserve(sample*2);
  ng = double(sample);
  double theta = 0, phi = 0;
  for (int i = 0; i < sample; i++)
  {
    double i_r8 = (double)(-ng + 1 + 2 * i);
    octomap::point3d point;
    octomap::point3d point2;
    theta = 2 * r_phi * i_r8 / M_PI;
    double sphi = i_r8 / ng;
    double cphi = sqrt((ng + i_r8) * (ng - i_r8)) / ng;

    point.x() = r * cphi * sin(theta) + origin.x();
    point.y() = r * cphi * cos(theta) + origin.x();
    point.z() = r * cphi;
    point2.z() = -point.z();
    point2.y() = -point.y();
    point2.x() = -point.x();
    spherePoints.push_back(point);
    spherePoints.push_back(point2);
    // cout<<point.x()<<" "<<point.y()<<" "<<point.z()<<endl;
    // cout<< i_r8<<endl;
  }
  return spherePoints;
}

double SphereDiscretization::r8_modp(double x, double y)
{
  // Returns non-negetive remainder of R8 division.
  double value;
  if (y = 0.0)
  {
    //cerr << "R8-MODP error: called with the y value of " << y << "\n";
    ROS_ERROR_STREAM("R8-M0DP error: called with the y value of " << y);
    exit(1);
  }
  value = x - ((double)((int)(x / y))) * y;
  if (value < 0.0)
  {
    value = value + fabs(y);
  }
  return value;
}

octomap::Pointcloud SphereDiscretization::make_sphere_spiral_points(const octomap::point3d& origin, double r, int sample)
{
  octomap::Pointcloud spherePoints;
  spherePoints.reserve(sample);

  double theta = 0, phi = 0;
  double sinphi = 0, cosphi = 0;
  for (int i = 0; i < sample; i++)
  {
    octomap::point3d point(0, 0, 0);
    cosphi = ((double)(sample - i - 1) * (-1.0) + (double)(i) * (1.0)) / (double)(sample - 1);
    sinphi = sqrt(1.0 - cosphi * cosphi);
    if (i == 0 || i == sample - 1)
    {
      theta = 0.0;
    }
    else
    {
      theta = theta + 3.6 / (sinphi * sqrt((double)sample));
      theta = r8_modp(theta, 2.0 * M_PI);
    }

    point.x() = r * sinphi * cos(theta) + origin.x();
    point.y() = r * sinphi * sin(theta) + origin.y();
    point.z() = r * cosphi + origin.y();

    spherePoints.push_back(point);

    // cout<<point.x()<<" "<<point.y()<<" "<<point.z()<<endl;
  }
  return spherePoints;
}

octomap::Pointcloud SphereDiscretization::make_long_lat_grid(const octomap::point3d& origin, double r, int sample, int lat_num, int lon_num)
{
  octomap::Pointcloud spherePoints;

  double theta = 0, phi = 0;
  octomap::point3d point;
  point.x() = r * sin(phi) * cos(theta) + origin.x();
  point.y() = r * sin(phi) * sin(theta) + origin.y();
  point.z() = r * cos(phi) + origin.y();
  for (int lat = 1; lat < lat_num; lat++)
  {
    phi = (double)(lat)*M_PI / (double)(lat_num + 1);
    for (int lon = 0; lon < lon_num; lon++)
    {
      {
        theta = (double)(lon)*2.0 * M_PI / (double)(lon_num);
        point.x() = r * sin(phi) * cos(theta) + origin.x();
        point.y() = r * sin(phi) * sin(theta) + origin.y();
        point.z() = r * cos(phi) + origin.y();
        sample = sample + 1;
        spherePoints.push_back(point);
      }
    }
    point.x() = r * sin(phi) * cos(theta) + origin.x();
    point.y() = r * sin(phi) * sin(theta) + origin.y();
    point.z() = r * cos(phi) + origin.y();
    sample = sample + 1;
    //cout << point.x() << " " << point.y() << " " << point.z() << endl;
    ROS_INFO_STREAM(point.x() << " " << point.y() << " " << point.z());
  }
  return spherePoints;
}

void SphereDiscretization::convertPointToVector(const octomap::point3d point, std::vector< double >& data)
{
  data.resize(3);
  data[0]=(double(point.x()));
  data[1]=(double(point.y()));
  data[2]=(double(point.z()));
}

void SphereDiscretization::convertVectorToPoint(const std::vector< double > data, octomap::point3d &point)
{
  point.x() = data[0];
  point.y() = data[1];
  point.z() = data[2];
}

void SphereDiscretization::convertPoseToVector(const geometry_msgs::Pose& pose, std::vector< double > &data)
{
  data.resize(7);
  data[0]=(double(pose.position.x));
  data[1]=(double(pose.position.y));
  data[2]=(double(pose.position.z));
  data[3]=(double(pose.orientation.x));
  data[4]=(double(pose.orientation.y));
  data[5]=(double(pose.orientation.z));
  data[6]=(double(pose.orientation.w));
}

void SphereDiscretization::convertVectorToPose(const std::vector< double >& data, geometry_msgs::Pose &pose)
{
  pose.position.x = data[0];
  pose.position.y = data[1];
  pose.position.z = data[2];
  pose.orientation.x = data[3];
  pose.orientation.y = data[4];
  pose.orientation.z = data[5];
  pose.orientation.w = data[6];
}

geometry_msgs::Pose SphereDiscretization::findOptimalPose(const std::vector< geometry_msgs::Pose >& poses, const octomap::point3d& origin)
{
  geometry_msgs::Pose optiPose;
  double mydo[] = {0, 0, 0, 0, 0, 0};
  std::vector< double > mean(mydo, mydo + sizeof(mydo) / sizeof(double));
  for (int i = 0; i < poses.size(); i++)
  {
    mean[0] += poses[i].position.x;
    mean[1] += poses[i].position.y;
    mean[2] += poses[i].position.z;
    mean[3] += poses[i].orientation.x;
    mean[4] += poses[i].orientation.y;
    mean[5] += poses[i].orientation.z;
    mean[6] += poses[i].orientation.w;
  }
  tf2::Quaternion quat(mean[3] / poses.size(), mean[4] / poses.size(), mean[5] / poses.size(), mean[6] / poses.size());
  quat.normalize();
  optiPose.position.x = origin.x();
  optiPose.position.y = origin.y();
  optiPose.position.z = origin.z();
  optiPose.orientation.x = quat.x();
  optiPose.orientation.y = quat.y();
  optiPose.orientation.z = quat.z();
  optiPose.orientation.w = quat.w();

  return optiPose;
}

void SphereDiscretization::createConeCloud(const geometry_msgs::Pose& pose, const double opening_angle,
                                           const double scale, pcl::PointCloud< pcl::PointXYZ >::Ptr cloud)
{
  pcl::PointCloud< pcl::PointXYZ >::Ptr new_cloud(new pcl::PointCloud< pcl::PointXYZ >);
  float angle = M_PI / opening_angle;
  static const double delta_theta = M_PI / 32.0;
  double theta = 0;
  octomap::point3d origin = octomap::point3d(0, 0, 0);
  for (float j = 0; j <= angle / 2; j += 0.01)
  {
    for (float k = 0; k <= scale; k += 0.001)
    {
      pcl::PointXYZ point;
      point.x = origin.x() + k;
      point.y = origin.y() + k * cos(theta + delta_theta) * j;
      point.z = origin.z() + k * sin(theta + delta_theta) * j;
      theta += delta_theta;
      new_cloud->push_back(point);
    }
  }

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  Eigen::Quaternionf q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  q.normalize();
  transform.translation() << pose.position.x, pose.position.y, pose.position.z;

  transform.rotate(q);
  transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()));
  pcl::transformPointCloud(*new_cloud, *cloud, transform);
}

void SphereDiscretization::poseToPoint(const geometry_msgs::Pose& pose, octomap::point3d &point)
{
  point.x() = pose.position.x;
  point.y() = pose.position.y;
  point.z() = pose.position.z;
}
bool SphereDiscretization::isPointInCloud(pcl::PointCloud< pcl::PointXYZ >::Ptr cloud, octomap::point3d point)
{
  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_hull(new pcl::PointCloud< pcl::PointXYZ >);
  pcl::ConvexHull< pcl::PointXYZ > chull;
  chull.setInputCloud(cloud);
  chull.setComputeAreaVolume(true);
  chull.reconstruct(*cloud_hull);
  pcl::PointXYZ query_point;
  query_point.x = float(point.x());
  query_point.y = float(point.y());
  query_point.z = float(point.z());
  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_new(new pcl::PointCloud< pcl::PointXYZ >);

  for (size_t j = 0; j < cloud->points.size(); ++j)
  {
    cloud_new->push_back(cloud->points[j]);
  }
  cloud_new->push_back(query_point);
  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud_hull_check(new pcl::PointCloud< pcl::PointXYZ >);
  pcl::ConvexHull< pcl::PointXYZ > chull_check;
  chull_check.setInputCloud(cloud_new);
  chull_check.setComputeAreaVolume(true);
  chull_check.reconstruct(*cloud_hull_check);
  if (chull_check.getTotalArea() - chull.getTotalArea() <= 0.05)
  {
    return true;
  }
  else
  {
    return false;
  }
}

float SphereDiscretization::distanceL2norm(const octomap::point3d& p1, const octomap::point3d& p2)
{
  return sqrt((p1.x() - p2.x()) * (p1.x() - p2.x()) +
              (p1.y() - p2.y()) * (p1.y() - p2.y()) +
              (p1.z() - p2.z()) * (p1.z() - p2.z()));
}

void SphereDiscretization::poseToEigenVector(const geometry_msgs::Pose& pose, Eigen::VectorXd& vec)
{
  vec << pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z,
      pose.orientation.w;
}

void SphereDiscretization::findOptimalPosebyPCA(const std::vector< geometry_msgs::Pose >& probBasePoses,
                                                geometry_msgs::Pose& final_base_pose)
{
  Eigen::Matrix4d M;
  Eigen::MatrixXd poseData(7, probBasePoses.size());
  for (int i = 0; i < probBasePoses.size(); ++i)
  {
    Eigen::VectorXd vec(7);
    Eigen::Vector4d q(probBasePoses[i].orientation.x, probBasePoses[i].orientation.y, probBasePoses[i].orientation.z, probBasePoses[i].orientation.w);
    poseToEigenVector(probBasePoses[i], vec);
    poseData.col(i) << vec;
    if (i == 0)
    {
      Eigen::Matrix4d D((q * q.transpose()));
      M = D/probBasePoses.size();
    }
    else
    {
      Eigen::Matrix4d D((q * q.transpose()));
      M += D/probBasePoses.size();
    }
  }

  Eigen::EigenSolver< Eigen::MatrixXd > eig(M);
  Eigen::VectorXd::Index idx;
  Eigen::VectorXd test = eig.eigenvalues().real();

  int i = test.maxCoeff(&idx);
  Eigen::Vector4d vector = eig.eigenvectors().col(idx).real();

  final_base_pose.position.x =0;
  final_base_pose.position.y =0;
  final_base_pose.position.z =0;

  final_base_pose.orientation.x = vector[0];
  final_base_pose.orientation.y = vector[1];
  final_base_pose.orientation.z = vector[2];
  final_base_pose.orientation.w = vector[3];
}

bool SphereDiscretization::areQuaternionClose(const tf2::Quaternion& q1, const tf2::Quaternion& q2)
{
  double dot = q1.dot(q2);
  return (dot >= 0);
}

tf2::Quaternion SphereDiscretization::inverseSignQuaternion(const tf2::Quaternion& q)
{
  return tf2::Quaternion(-q[0], -q[1], -q[2], -q[3]);
}

void SphereDiscretization::findOptimalPosebyAverage(const std::vector< geometry_msgs::Pose > probBasePoses,
                                                    geometry_msgs::Pose& final_base_pose)
{
  // This Function has been borrowed from http://wiki.unity3d.com/index.php/Averaging_Quaternions_and_Vectors

  double totalVecX, totalVecY, totalVecZ;
  double avgVecX, avgVecY, avgVecZ;
  std::vector< tf2::Quaternion > quatCol;
  quatCol.reserve(probBasePoses.size());

  for (int i = 0; i < probBasePoses.size(); ++i)
  {
    totalVecX += probBasePoses[i].position.x;
    totalVecY += probBasePoses[i].position.y;
    totalVecZ += probBasePoses[i].position.z;
    tf2::Quaternion quats(probBasePoses[i].orientation.x, probBasePoses[i].orientation.y,
                          probBasePoses[i].orientation.z, probBasePoses[i].orientation.w);
    quatCol.push_back(quats);
  }

  avgVecX = totalVecX / double(probBasePoses.size());
  avgVecY = totalVecY / double(probBasePoses.size());
  avgVecZ = totalVecZ / double(probBasePoses.size());

  double totalQuatX, totalQuatY, totalQuatZ, totalQuatW;
  double avgQuatX, avgQuatY, avgQuatZ, avgQuatW;
  for (int j = 0; j < quatCol.size(); ++j)
  {
    if (!areQuaternionClose(quatCol[0], quatCol[j]))
    {
      tf2::Quaternion quat_new;
      quat_new = inverseSignQuaternion(quatCol[j]);

      totalQuatX += quat_new[0];
      totalQuatY += quat_new[1];
      totalQuatZ += quat_new[2];
      totalQuatW += quat_new[3];
    }
    else
    {
      totalQuatX += quatCol[j][0];
      totalQuatY += quatCol[j][1];
      totalQuatZ += quatCol[j][2];
      totalQuatW += quatCol[j][3];
    }
  }
  avgQuatX = totalQuatX / double(probBasePoses.size());
  avgQuatY = totalQuatY / double(probBasePoses.size());
  avgQuatZ = totalQuatZ / double(probBasePoses.size());
  avgQuatW = totalQuatW / double(probBasePoses.size());
  tf2::Quaternion final_base_quat(avgQuatX, avgQuatY, avgQuatZ, avgQuatW);
  final_base_quat.normalize();

  final_base_pose.position.x = avgVecX;
  final_base_pose.position.y = avgVecY;
  final_base_pose.position.z = avgVecZ;
  final_base_pose.orientation.x = final_base_quat[0];
  final_base_pose.orientation.y = final_base_quat[1];
  final_base_pose.orientation.z = final_base_quat[2];
  final_base_pose.orientation.w = final_base_quat[3];
}

void SphereDiscretization::associatePose(std::multimap< std::vector< double >, std::vector< double > >& baseTrnsCol,
                                         const std::vector< geometry_msgs::Pose >& grasp_poses,
                                         const std::multimap< std::vector< double >, std::vector< double > >& PoseColFilter,
                                         const float resolution)
{
  unsigned char maxDepth = 16;
  float size_of_box = 1.5;
  SphereDiscretization sd;
  octomap::point3d origin = octomap::point3d(0, 0, 0);
  octomap::OcTree* tree = sd.generateBoxTree(origin, size_of_box, resolution);
  std::vector< octomap::point3d > spCenter;
  for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(maxDepth), end = tree->end_leafs(); it != end; ++it)
  {
    spCenter.push_back(it.getCoordinate());
  }

  // create a point cloud which consists of all of the possible base locations for all grasp poses and a list of base
  // pose orientations
  std::vector< std::pair< std::vector< float >, std::vector< float > > > trns_col;
  trns_col.reserve(grasp_poses.size());
  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZ >);
  for (int i = 0; i < grasp_poses.size(); ++i)
  {
    // get grasp pose in tf format
    tf2::Vector3 grasp_vec(grasp_poses[i].position.x, grasp_poses[i].position.y, grasp_poses[i].position.z);
    tf2::Quaternion grasp_quat(grasp_poses[i].orientation.x, grasp_poses[i].orientation.y, grasp_poses[i].orientation.z,
                               grasp_poses[i].orientation.w);
    grasp_quat.normalize();
    tf2::Transform grasp_trns;
    grasp_trns.setOrigin(grasp_vec);
    grasp_trns.setRotation(grasp_quat);

    // iterate through the inverse reach map
    for (std::multimap< std::vector< double >, std::vector< double > >::const_iterator it = PoseColFilter.begin();
         it != PoseColFilter.end(); ++it)
    {
      tf2::Vector3 vec(it->second[0], it->second[1], it->second[2]);
      tf2::Quaternion quat(it->second[3], it->second[4], it->second[5], it->second[6]);
      tf2::Transform trns;
      trns.setOrigin(vec);
      trns.setRotation(quat);

      tf2::Transform new_trns;
      new_trns = grasp_trns * trns;

      tf2::Vector3 new_trans_vec;
      tf2::Quaternion new_trans_quat;
      new_trans_vec = new_trns.getOrigin();
      new_trans_quat = new_trns.getRotation();
      new_trans_quat.normalize();

      std::vector< float > position;
      position.reserve(3);
      position.push_back(new_trans_vec[0]);
      position.push_back(new_trans_vec[1]);
      position.push_back(new_trans_vec[2]);
      std::vector< float > orientation;
      position.reserve(4);

      orientation.push_back(new_trans_quat[0]);
      orientation.push_back(new_trans_quat[1]);
      orientation.push_back(new_trans_quat[2]);
      orientation.push_back(new_trans_quat[3]);
      trns_col.push_back(std::pair< std::vector< float >, std::vector< float > >(position, orientation));

      pcl::PointXYZ point;
      point.x = new_trans_vec[0];
      point.y = new_trans_vec[1];
      point.z = new_trans_vec[2];
      cloud->push_back(point);
    }
  }  // done creating base pose cloud

  // Create octree for binning the base poses
  pcl::octree::OctreePointCloudSearch< pcl::PointXYZ > octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  // add all base poses from cloud to an octree
  for (int i = 0; i < spCenter.size(); i++)
  {
    pcl::PointXYZ searchPoint;
    searchPoint.x = spCenter[i].x();
    searchPoint.y = spCenter[i].y();
    searchPoint.z = spCenter[i].z();

    // Find all base poses that lie in the given voxel
    std::vector< int > pointIdxVec;
    octree.voxelSearch(searchPoint, pointIdxVec);
    if (pointIdxVec.size() > 0)
    {
      std::vector< double > voxel_pos;
      voxel_pos.reserve(3);
      voxel_pos.push_back(searchPoint.x);
      voxel_pos.push_back(searchPoint.y);
      voxel_pos.push_back(searchPoint.z);

      // For a given voxel, add all base poses to the multimap for later retreival
      for (size_t j = 0; j < pointIdxVec.size(); ++j)
      {
        // Get the base pose for a given index found in a voxel
        std::vector< double > base_pose;
        base_pose.reserve(3);
        base_pose.push_back(voxel_pos[0]);
        base_pose.push_back(voxel_pos[1]);
        base_pose.push_back(voxel_pos[2]);
        std::vector< float > orientation = trns_col[pointIdxVec[j]].second;
        base_pose.push_back(double(orientation[0]));
        base_pose.push_back(double(orientation[1]));
        base_pose.push_back(double(orientation[2]));
        base_pose.push_back(double(orientation[3]));

        baseTrnsCol.insert(std::pair< std::vector< double >, std::vector< double > >(voxel_pos, base_pose));
      }
    }
  }
}
}
