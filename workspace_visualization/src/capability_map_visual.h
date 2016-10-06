#ifndef CAPABILITY_MAP_VISUAL_H
#define CAPABILITY_MAP_VISUAL_H

#include <map_creator/capability.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Shape;
}

namespace workspace_visualization
{
class CapMapDisplay;
class CapMapVisual
{
public:
  CapMapVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, rviz::DisplayContext* display);
  virtual ~CapMapVisual();
  void setMessage(const map_creator::capability::ConstPtr& msg, int low_ri, int high_ri, int disect_choice);
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setColor(float r, float g, float b, float a);
  void setSize(float l);

  void setColorbyRI(float alpha);

private:
  std::vector< boost::shared_ptr< rviz::Shape > > sphere_;
  std::vector< int > colorRI_;

  Ogre::SceneNode* frame_node_;

  Ogre::SceneManager* scene_manager_;
};
}  // end namespace workspace_visualization
#endif  // CAPABILITY_MAP_VISUAL_H
