#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/frame_manager.h>

#include "reachability_map_visual.h"

#include "reachability_map_display.h"

namespace workspace_visualization
{
ReachMapDisplay::ReachMapDisplay()
{
  do_display_arrow_ = new rviz::BoolProperty("Show Poses", false, "Displays the arrow.", this);
  do_display_sphere_ = new rviz::BoolProperty("Show Shape", true, "Displays the spheres.", this);
  is_byReachability_ = new rviz::BoolProperty("Color by Reachability", true, "Color transform by Reachability Index", this);

  shape_property_ = new rviz::EnumProperty("Shape", "Sphere", "Shape to display the workspace.", this,
                                           SLOT(updateColorAndAlphaArrow()));
  shape_property_->addOption("Sphere", Sphere);
  shape_property_->addOption("Cylinder", Cylinder);
  shape_property_->addOption("Cone", Cone);
  shape_property_->addOption("Cube", Cube);

  disect_property_ =
      new rviz::EnumProperty("Disect", "Full", "Disection of the workspace", this, SLOT(updateColorAndAlphaArrow()));
  disect_property_->addOption("Full", Full);
  disect_property_->addOption("1st_Half", First_Half);
  disect_property_->addOption("2nd_Half", Second_Half);
  disect_property_->addOption("Middle_Slice", Middle_Slice);
  disect_property_->addOption("End_Slice", End_Slice);

  // Arrow Property category
  arrow_category_ = new rviz::Property("Poses Property", QVariant(), "", this);

  arrow_color_property_ = new rviz::ColorProperty("Color", QColor(204, 51, 204), "Color to draw the Pose arrows.",
                                                  arrow_category_, SLOT(updateColorAndAlphaArrow()), this);

  arrow_alpha_property_ = new rviz::FloatProperty("Alpha", 0.2, "0 is fully transparent, 1.0 is fully opaque.",
                                                  arrow_category_, SLOT(updateColorAndAlphaArrow()), this);

  arrow_length_property_ =
      new rviz::FloatProperty("Length", 0.01, "Length of the arrows", arrow_category_, SLOT(updateArrowSize()), this);

  // Shape Property category

  sphere_category_ = new rviz::Property("Shape Property", QVariant(), "", this);

  sphere_color_property_ = new rviz::ColorProperty("Color", QColor(255, 225, 102), "Color to draw the Sphere.",
                                                   sphere_category_, SLOT(updateColorAndAlphaSphere()), this);

  sphere_alpha_property_ = new rviz::FloatProperty("Alpha", 1.0, "0 is fully transparent, 1.0 is fully opaque.",
                                                   sphere_category_, SLOT(updateColorAndAlphaSphere()), this);

  sphere_radius_property_ =
      new rviz::FloatProperty("Size", 0.05, "Size of the sphere", sphere_category_, SLOT(updateSphereSize()), this);

  lower_bound_reachability_ = new rviz::IntProperty("Lowest Reachability Index", 0, "Lowest Reachability index.", this);

  upper_bound_reachability_ =
      new rviz::IntProperty("Highest Reachability Index", 100, "Highest Reachability index.", this);
}

void ReachMapDisplay::onInitialize()
{
  MFDClass::onInitialize();
}

ReachMapDisplay::~ReachMapDisplay()
{
}
void ReachMapDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}

void ReachMapDisplay::updateColorAndAlphaArrow()
{
  float alpha = arrow_alpha_property_->getFloat();
  Ogre::ColourValue color = arrow_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setColorArrow(color.r, color.g, color.b, alpha);
  }
}

void ReachMapDisplay::updateArrowSize()
{
  float length = arrow_length_property_->getFloat();
  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setSizeArrow(length);
  }
}

void ReachMapDisplay::updateColorAndAlphaSphere()
{
  float alpha = sphere_alpha_property_->getFloat();
  Ogre::ColourValue color = sphere_color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setColorSphere(color.r, color.g, color.b, alpha);
  }
}

void ReachMapDisplay::updateSphereSize()
{
  float length = sphere_radius_property_->getFloat();
  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[i]->setSizeSphere(length);
  }
}

void ReachMapDisplay::processMessage(const map_creator::WorkSpace::ConstPtr& msg)
{
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }
  boost::shared_ptr< ReachMapVisual > visual;
  visual.reset(new ReachMapVisual(context_->getSceneManager(), scene_node_, context_));

  visual->setMessage(msg, do_display_arrow_->getBool(), do_display_sphere_->getBool(),
                     lower_bound_reachability_->getInt(), upper_bound_reachability_->getInt(),
                     shape_property_->getOptionInt(), disect_property_->getOptionInt());

  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  float arrow_alpha = arrow_alpha_property_->getFloat();
  Ogre::ColourValue arrow_color = arrow_color_property_->getOgreColor();
  visual->setColorArrow(arrow_color.r, arrow_color.g, arrow_color.b, arrow_alpha);

  float sphere_alpha = sphere_alpha_property_->getFloat();
  Ogre::ColourValue sphere_color = sphere_color_property_->getOgreColor();
  if (is_byReachability_->getBool() == false)
  {
    visual->setColorSphere(sphere_color.r, sphere_color.g, sphere_color.b, sphere_alpha);
  }
  else
  {
    visual->setColorSpherebyRI(sphere_alpha);
  }

  visuals_.push_back(visual);
  //updateArrowSize();
  updateSphereSize();

}

}  // end namespace workspace_visualization
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(workspace_visualization::ReachMapDisplay, rviz::Display)
