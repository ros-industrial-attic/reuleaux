#ifndef CAPABILITY_MAP_DISPLAY_H
#define CAPABILITY_MAP_DISPLAY_H

#ifndef Q_MOC_RUN

#include <rviz/message_filter_display.h>

#include <map_creator/capability.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class EnumProperty;
class BoolProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace workspace_visualization
{
class CapMapVisual;
class CapMapDisplay : public rviz::MessageFilterDisplay< map_creator::capability >
{
  Q_OBJECT
public:
  enum Disect
  {
    Full,
    First_Half,
    Second_Half,
    Middle_Slice,
    End_Slice,
  };
  CapMapDisplay();
  virtual ~CapMapDisplay();

protected:
  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateSize();

private:
  void processMessage(const map_creator::capability::ConstPtr& msg);
  std::vector< boost::shared_ptr< CapMapVisual > > visuals_;

  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* radius_property_;
  rviz::IntProperty* lower_bound_reachability_;
  rviz::IntProperty* upper_bound_reachability_;
  rviz::BoolProperty* is_byReachability_;
  rviz::EnumProperty* disect_property_;
};

}  // end namespace workspace_visualization
#endif  // CAPABILITY_MAP_DISPLAY_H
