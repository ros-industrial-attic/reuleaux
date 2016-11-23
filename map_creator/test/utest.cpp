/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	* Redistributions of source code must retain the above copyright
 * 	notice, this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	notice, this list of conditions and the following disclaimer in the
 * 	documentation and/or other materials provided with the distribution.
 * 	* Neither the name of the Southwest Research Institute, nor the names
 *	of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <map_creator/sphere_discretization.h>
#include <gtest/gtest.h>

using namespace sphere_discretization;
//using sphere_discretization::findOptimalPosebyPCA;

TEST(PCATest, TestCase1)
{
  sphere_discretization::SphereDiscretization sd;

  const double DEFAULT_ALLOWED_VARATION = 0.01;
  std::vector< geometry_msgs::Pose > poses;
  geometry_msgs::Pose pose, pose1, pose2, final;

  pose1.position.x = 0;
  pose1.position.y = 0;
  pose1.position.z = 0;

  // normal unit quaternion (aligned with x-axis)
  pose1.orientation.x = 0;
  pose1.orientation.y = 0;
  pose1.orientation.z = 0;
  pose1.orientation.w = 1;

  poses.push_back(pose1);
  pose = pose1;
  // 90 degree rotation around y-axis (aligned with z-axis)
  pose1.orientation.x = 0;
  pose1.orientation.y = 0.707;
  pose1.orientation.z = 0;
  pose1.orientation.w = 0.707;

  poses.push_back(pose1);

  // final pose
  pose2 = pose1;
  pose2.orientation.x = 0.0;
  pose2.orientation.y = 0.383;
  pose2.orientation.z = 0.0;
  pose2.orientation.w = 0.924;

  sd.findOptimalPosebyPCA(poses, final);
  EXPECT_NEAR(final.orientation.x, pose2.orientation.x, DEFAULT_ALLOWED_VARATION);
  EXPECT_NEAR(final.orientation.y, pose2.orientation.y, DEFAULT_ALLOWED_VARATION);
  EXPECT_NEAR(final.orientation.z, pose2.orientation.z, DEFAULT_ALLOWED_VARATION);
  EXPECT_NEAR(final.orientation.w, pose2.orientation.w, DEFAULT_ALLOWED_VARATION);

  // 90 degree rotation around y-axis (aligned with z-axis)
  pose1.orientation.x = 0;
  pose1.orientation.y = -0.707;
  pose1.orientation.z = 0;
  pose1.orientation.w = 0.707;

  poses.push_back(pose1);

  sd.findOptimalPosebyPCA(poses, final);
  EXPECT_NEAR(final.orientation.x, pose.orientation.x, DEFAULT_ALLOWED_VARATION);
  EXPECT_NEAR(final.orientation.y, pose.orientation.y, DEFAULT_ALLOWED_VARATION);
  EXPECT_NEAR(final.orientation.z, pose.orientation.z, DEFAULT_ALLOWED_VARATION);
  EXPECT_NEAR(final.orientation.w, pose.orientation.w, DEFAULT_ALLOWED_VARATION);

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");  // some tests need ROS framework
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


