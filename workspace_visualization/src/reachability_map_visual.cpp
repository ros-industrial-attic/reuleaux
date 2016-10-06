#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/display_context.h>
#include <rviz/display_factory.h>

#include <tf2/LinearMath/Quaternion.h>

#include "reachability_map_visual.h"
#include <iterator>

namespace workspace_visualization
{
ReachMapVisual::ReachMapVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                               rviz::DisplayContext* display_context)
{
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();

  // arrow_.reset(new rviz::Arrow( scene_manager_, frame_node_ ));
}

ReachMapVisual::~ReachMapVisual()
{
  scene_manager_->destroySceneNode(frame_node_);
}

void ReachMapVisual::setMessage(const map_creator::WorkSpace::ConstPtr& msg, bool do_display_arrow,
                                bool do_display_sphere, int low_ri, int high_ri, int shape_choice, int disect_choice)
{
  int low_SphereSize, up_SphereSize;
  // TODO: Not a very delicate process to dissect the workspace. Implement a way to provide a range and allow the user
  // to select an axis and an upper/lower bound to slice with
  switch (disect_choice)
  {
    case 0:
    {
      low_SphereSize = 0;
      up_SphereSize = msg->WsSpheres.size();
      break;
    }
    case 1:
    {
      low_SphereSize = 0;
      up_SphereSize = msg->WsSpheres.size() / 2;
      break;
    }
    case 2:
    {
      low_SphereSize = msg->WsSpheres.size() / 2;
      up_SphereSize = msg->WsSpheres.size();
      break;
    }
    case 3:
    {
      low_SphereSize = msg->WsSpheres.size() / 2.2;
      up_SphereSize = msg->WsSpheres.size() / 1.8;
      break;
    }
    case 4:
    {
      low_SphereSize = 0;
      up_SphereSize = msg->WsSpheres.size() / 1.1;
      break;
    }
  }

  if (do_display_arrow)
  {
    boost::shared_ptr< rviz::Arrow > pose_arrow;
    for (size_t i = low_SphereSize; i < up_SphereSize; ++i)
    {
      for (size_t j = 0; j < msg->WsSpheres[i].poses.size(); ++j)
      {
        if (low_ri < int(msg->WsSpheres[i].ri) && int(msg->WsSpheres[i].ri) <= high_ri)
        {
          pose_arrow.reset(new rviz::Arrow(scene_manager_, frame_node_));

          Ogre::Vector3 position_(msg->WsSpheres[i].poses[j].position.x, msg->WsSpheres[i].poses[j].position.y,
                                  msg->WsSpheres[i].poses[j].position.z);
          tf2::Quaternion quat(msg->WsSpheres[i].poses[j].orientation.x, msg->WsSpheres[i].poses[j].orientation.y,
                               msg->WsSpheres[i].poses[j].orientation.z, msg->WsSpheres[i].poses[j].orientation.w);

          tf2::Quaternion q2;
          q2.setRPY(0, -M_PI / 2, 0);  // Arrows are pointed as -z direction. So rotating it is necessary

          quat *= (q2);
          quat.normalize();
          Ogre::Quaternion orientation_(quat.w(), quat.x(), quat.y(), quat.z());

          if (position_.isNaN() || orientation_.isNaN())
          {
            ROS_WARN("received invalid pose");
            return;
          }

          pose_arrow->setPosition(position_);
          pose_arrow->setOrientation(orientation_);

          arrow_.push_back(pose_arrow);
        }
      }
    }
  }
  if (do_display_sphere)
  {
    boost::shared_ptr< rviz::Shape > sphere_center;
    int colorRI;

    for (size_t i = low_SphereSize; i < up_SphereSize; ++i)
    {
      if (low_ri <= int(msg->WsSpheres[i].ri) && int(msg->WsSpheres[i].ri <= high_ri))
      {
        switch (shape_choice)
        {
          case 0:
          {
            sphere_center.reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
            break;
          }
          case 1:
          {
            sphere_center.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, frame_node_));
            break;
          }
          case 2:
          {
            sphere_center.reset(new rviz::Shape(rviz::Shape::Cone, scene_manager_, frame_node_));
            break;
          }
          case 3:
          {
            sphere_center.reset(new rviz::Shape(rviz::Shape::Cube, scene_manager_, frame_node_));
            break;
          }
        }

        Ogre::Vector3 position_sphere(msg->WsSpheres[i].point.x, msg->WsSpheres[i].point.y, msg->WsSpheres[i].point.z);
        Ogre::Quaternion orientation_sphere(0, 0, 0, 1);
        colorRI = msg->WsSpheres[i].ri;
        if (position_sphere.isNaN())
        {
          ROS_WARN("received invalid sphere coordinate");
          return;
        }
        sphere_center->setPosition(position_sphere);
        sphere_center->setOrientation(orientation_sphere);

        sphere_.push_back(sphere_center);
        colorRI_.push_back(colorRI);
      }
    }
  }
}

void ReachMapVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void ReachMapVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

void ReachMapVisual::setColorArrow(float r, float g, float b, float a)
{
  for (int i = 0; i < arrow_.size(); ++i)
  {
    arrow_[i]->setColor(r, g, b, a);
  }
}

void ReachMapVisual::setColorSphere(float r, float g, float b, float a)
{
  for (int i = 0; i < sphere_.size(); ++i)
  {
    sphere_[i]->setColor(r, g, b, a);
  }
}

void ReachMapVisual::setColorSpherebyRI(float alpha)
{
  for (int i = 0; i < sphere_.size(); ++i)
  {
    if (colorRI_[i] >= 90)
    {
      sphere_[i]->setColor(0, 0, 255, alpha);
    }
    else if (colorRI_[i] < 90 && colorRI_[i] >= 50)
    {
      sphere_[i]->setColor(0, 255, 255, alpha);
    }
    else if (colorRI_[i] < 50 && colorRI_[i] >= 30)
    {
      sphere_[i]->setColor(0, 255, 0, alpha);
    }
    else if (colorRI_[i] < 30 && colorRI_[i] >= 5)
    {
      sphere_[i]->setColor(255, 255, 0, alpha);
    }
    else
    {
      sphere_[i]->setColor(255, 0, 0, alpha);
    }
  }
}

void ReachMapVisual::setSizeArrow(float l)
{
  for (int i = 0; i < arrow_.size(); ++i)
  {
    arrow_[i]->setScale(Ogre::Vector3(l, l, l));
  }
}

void ReachMapVisual::setSizeSphere(float l)
{
  for (int i = 0; i < sphere_.size(); ++i)
  {
    sphere_[i]->setScale(Ogre::Vector3(l, l, l));
  }
}

}  // end namespace workspace_visualization
