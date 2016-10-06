#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/frame_manager.h>

#include "capability_map_visual.h"

#include "capability_map_display.h"

namespace workspace_visualization
{
CapMapDisplay::CapMapDisplay()
{
  is_byReachability_ =
      new rviz::BoolProperty("Color by Reachability", true, "Color transform by Reachability Index", this);
  disect_property_ =
      new rviz::EnumProperty("Disect", "Full", "Disection of the workspace", this, SLOT(updateColorAndAlpha()));
  disect_property_->addOption("Full", Full);
  disect_property_->addOption("1st_Half", First_Half);
  disect_property_->addOption("2nd_Half", Second_Half);
  disect_property_->addOption("Middle_Slice", Middle_Slice);
  disect_property_->addOption("End_Slice", End_Slice);

  color_property_ = new rviz::ColorProperty("Color", QColor(255, 225, 102), "Color to draw the Sphere.", this,
                                            SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty("Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.", this,
                                            SLOT(updateColorAndAlpha()));

  radius_property_ = new rviz::FloatProperty("Size", 0.1, "Size of the sphere", this, SLOT(updateSize()));

  lower_bound_reachability_ = new rviz::IntProperty("Lowest Reachability Index", 0, "Lowest Reachability index.", this);

  upper_bound_reachability_ =
      new rviz::IntProperty("Highest Reachability Index", 100, "Highest Reachability index.", this);
}

void CapMapDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

CapMapDisplay::~CapMapDisplay()
{
}

void CapMapDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void CapMapDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setColor(color.r, color.g, color.b, alpha);
  }
}

void CapMapDisplay::updateSize()
{
  float length = radius_property_->getFloat();
  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setSize(length);
  }
}

void CapMapDisplay::processMessage(const map_creator::capability::ConstPtr& msg)
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }
  boost::shared_ptr< CapMapVisual > visual;
  visual.reset(new CapMapVisual(context_->getSceneManager(), scene_node_, context_));
  visual->setMessage(msg, lower_bound_reachability_->getInt(), upper_bound_reachability_->getInt(),
                     disect_property_->getOptionInt());
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  if (is_byReachability_->getBool() == false)
  {
    visual->setColor(color.r, color.g, color.b, alpha);
  }
  else
  {
    visual->setColorbyRI(alpha);
  }
  visuals_.push_back(visual);
  updateSize();
}

}  // end namespace workspace_visualization
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(workspace_visualization::CapMapDisplay, rviz::Display)
