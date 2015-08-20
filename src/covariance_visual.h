// -*- c++-mode -*-
#ifndef COVARIANCE_VISUAL_H
#define COVARIANCE_VISUAL_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace Ogre
{
    class Vector3;
    class Quaternion;
}

namespace rviz
{
    class Axes;
    class Shape;
}

namespace rviz_plugin_covariance
{
    class CovarianceVisual
    {
        public:
            CovarianceVisual (Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);

            virtual ~CovarianceVisual ();

            void setMessage(const geometry_msgs::PoseWithCovariance& msg);
            void setMessage(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

            void setFramePosition(const Ogre::Vector3& position);
            void setFrameOrientation(const Ogre::Quaternion& orientation);

            void setColorPosition(float r, float g, float b, float a);
            void setColorOrientation(float r, float g, float b, float a);

            void setScaleCovariance(float scale) { scaleFactor_covariance_ = scale; }
            void setScaleAxis(float scale) { scaleFactor_axis_ = scale; }
        private:
            boost::shared_ptr<rviz::Axes> axes_;
            boost::shared_ptr<rviz::Shape> shape_;
            boost::shared_ptr<rviz::Shape> orientationShape_;

            Ogre::SceneNode* frame_node_;
            Ogre::SceneNode* positionNode_;
            Ogre::SceneNode* orientationNode_;

            Ogre::SceneManager* scene_manager_;

            float scaleFactor_covariance_;
            float scaleFactor_axis_;
    };
} // end namespace rviz_plugin_covariance

#endif // COVARIANCE_VISUAL_H
