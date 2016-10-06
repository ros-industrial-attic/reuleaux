#ifndef ReachMap_VISUAL_H
#define ReachMap_VISUAL_H

#include <map_creator/WorkSpace.h>

namespace Ogre
{
class Vector3;
class Quaternion;
}

namespace rviz
{
class Arrow;
class Shape;
}

namespace workspace_visualization
{
class ReachMapDisplay;
class ReachMapVisual
{
public:
  ReachMapVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, rviz::DisplayContext* display);
  virtual ~ReachMapVisual();
  void setMessage(const map_creator::WorkSpace::ConstPtr& msg, bool do_display_arrow, bool do_display_sphere,
                  int low_ri, int high_ri, int shape_choice, int disect_choice);
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);

  void setColorArrow(float r, float g, float b, float a);
  void setSizeArrow(float l);

  void setColorSphere(float r, float g, float b, float a);
  void setSizeSphere(float l);
  void setColorSpherebyRI(float alpha);

private:
  std::vector< boost::shared_ptr< rviz::Arrow > > arrow_;
  std::vector< boost::shared_ptr< rviz::Shape > > sphere_;
  std::vector< int > colorRI_;

  Ogre::SceneNode* frame_node_;

  Ogre::SceneManager* scene_manager_;
};
}  // end namespace workspace_visualization
#endif  // ReachMap_VISUAL_H
