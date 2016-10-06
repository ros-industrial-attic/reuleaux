#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>
#include <rviz/display_context.h>
#include <rviz/display_factory.h>

#include <tf2/LinearMath/Quaternion.h>

#include "capability_map_visual.h"

namespace workspace_visualization
{
CapMapVisual::CapMapVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node,
                           rviz::DisplayContext* display_context)
{
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
}

CapMapVisual::~CapMapVisual()
{
  scene_manager_->destroySceneNode(frame_node_);
}

void CapMapVisual::setMessage(const map_creator::capability::ConstPtr& msg, int low_ri, int high_ri, int disect_choice)
{
  int low_SphereSize, up_SphereSize;
  switch (disect_choice)
  {
    case 0:
    {
      low_SphereSize = 0;
      up_SphereSize = msg->capShapes.size();
      break;
    }
    case 1:
    {
      low_SphereSize = 0;
      up_SphereSize = msg->capShapes.size() / 2;
      break;
    }
    case 2:
    {
      low_SphereSize = msg->capShapes.size() / 2;
      up_SphereSize = msg->capShapes.size();
      break;
    }
    case 3:
    {
      low_SphereSize = msg->capShapes.size() / 2.2;
      up_SphereSize = msg->capShapes.size() / 1.8;
      break;
    }
    case 4:
    {
      low_SphereSize = 0;
      up_SphereSize = msg->capShapes.size() / 1.1;
      break;
    }
  }

  boost::shared_ptr< rviz::Shape > sphere_center;
  int colorRI;
  for (size_t i = low_SphereSize; i < up_SphereSize; ++i)
  {
    if (low_ri < int(msg->capShapes[i].ri) && int(msg->capShapes[i].ri) <= high_ri)
    {
      if (msg->capShapes[i].identifier == 2.0)
      {
        sphere_center.reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
      }

      else
      {
        sphere_center.reset(new rviz::Shape(rviz::Shape::Cone, scene_manager_, frame_node_));
      }

      Ogre::Vector3 position_(msg->capShapes[i].pose.position.x, msg->capShapes[i].pose.position.y,
                              msg->capShapes[i].pose.position.z);
      tf2::Quaternion quat(msg->capShapes[i].pose.orientation.x, msg->capShapes[i].pose.orientation.y,
                           msg->capShapes[i].pose.orientation.z, msg->capShapes[i].pose.orientation.w);

      quat.normalize();
      Ogre::Quaternion orientation_(quat.w(), quat.x(), quat.y(), quat.z());

      if (position_.isNaN() || orientation_.isNaN())
      {
        ROS_WARN("received invalid pose");
        return;
      }
      sphere_center->setPosition(position_);
      sphere_center->setOrientation(orientation_);

      sphere_.push_back(sphere_center);
      colorRI = msg->capShapes[i].ri;
      colorRI_.push_back(colorRI);
    }
  }
}

void CapMapVisual::setFramePosition(const Ogre::Vector3& position)
{
  frame_node_->setPosition(position);
}

void CapMapVisual::setFrameOrientation(const Ogre::Quaternion& orientation)
{
  frame_node_->setOrientation(orientation);
}

void CapMapVisual::setColor(float r, float g, float b, float a)
{
  for (int i = 0; i < sphere_.size(); ++i)
  {
    sphere_[i]->setColor(r, g, b, a);
  }
}

void CapMapVisual::setColorbyRI(float alpha)
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

void CapMapVisual::setSize(float l)
{
  for (int i = 0; i < sphere_.size(); ++i)
  {
    sphere_[i]->setScale(Ogre::Vector3(l, l, l));
  }
}

}  // end namespace workspace_visualization
