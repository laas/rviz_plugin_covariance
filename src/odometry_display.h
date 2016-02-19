// -*- c++-mode -*-
#ifndef ODOMETRY_DISPLAY_H
#define ODOMETRY_DISPLAY_H

#include <nav_msgs/Odometry.h>
#include <rviz/message_filter_display.h>

namespace Ogre
{
    class SceneNode;
}

namespace rviz
{
    class ColorProperty;
    class FloatProperty;
}

namespace rviz_plugin_covariance
{
    class PoseWithCovarianceVisual;

    class OdometryDisplay: public rviz::MessageFilterDisplay<nav_msgs::Odometry>
    {
        Q_OBJECT
        public:
            OdometryDisplay();
            virtual ~OdometryDisplay();

        protected:
            virtual void onInitialize();
            virtual void reset();

        private Q_SLOTS:
            void updateColorAndAlphaAndScale();

        private:
            void processMessage(const nav_msgs::Odometry::ConstPtr& msg);

            boost::shared_ptr<PoseWithCovarianceVisual> visual_;

            rviz::ColorProperty* color_property_;
            rviz::FloatProperty* alpha_property_;
            rviz::FloatProperty* scale_property_;
    };
} // end namespace rviz_plugin_covariance

#endif // ODOMETRY_DISPLAY_H
