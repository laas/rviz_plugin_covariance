#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/frame_manager.h>

#include "covariance_visual.h"

#include "covariance_display.h"

namespace rviz_plugin_covariance
{
    CovarianceDisplay::CovarianceDisplay()
    {
        color_property_ = new rviz::ColorProperty("Color", QColor( 204, 51, 204 ),
                                                  "Color to draw the covariance ellipse.",
                                                  this, SLOT(updateColorAndAlphaAndScale()));

        alpha_property_ = new rviz::FloatProperty("Alpha", 0.5f,
                                                  "0 is fully transparent, 1.0 is fully opaque.",
                                                  this, SLOT(updateColorAndAlphaAndScale()));

        scale_property_ = new rviz::FloatProperty("Scale", 1.0f,
                                                  "Scale factor to be applied to covariance ellipse",
                                                  this, SLOT(updateColorAndAlphaAndScale()));
    }

    void CovarianceDisplay::onInitialize()
    {
        MFDClass::onInitialize();
    }

    CovarianceDisplay::~CovarianceDisplay()
    {
    }

    void CovarianceDisplay::reset()
    {
        MFDClass::reset();
        //visual_.clear();
    }

    void CovarianceDisplay::updateColorAndAlphaAndScale()
    {
        Ogre::ColourValue color = color_property_->getOgreColor();
        float alpha = alpha_property_->getFloat();
        float scale = scale_property_->getFloat();

        if (visual_)
        {
            visual_->setColor(color.r, color.g, color.b, alpha);
            visual_->setScale(scale);
        }
    }

    void CovarianceDisplay::processMessage(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        Ogre::Quaternion orientation;
        Ogre::Vector3 position;

        if(!context_->getFrameManager()->getTransform(msg->header.frame_id,
                                                      msg->header.stamp,
                                                      position, orientation ))
        {
            ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
            msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
            return;
        }

        if (!visual_)
            visual_.reset(new CovarianceVisual(context_->getSceneManager(), scene_node_));

        visual_->setMessage (msg);
        visual_->setFramePosition (position);
        visual_->setFrameOrientation (orientation);

        Ogre::ColourValue color = color_property_->getOgreColor();
        float alpha = alpha_property_->getFloat();
        float scale = scale_property_->getFloat();

        visual_->setColor(color.r, color.g, color.b, alpha);
        visual_->setScale(scale);
    }
} // end namespace rviz_plugin_covariance

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_covariance::CovarianceDisplay, rviz::Display)
