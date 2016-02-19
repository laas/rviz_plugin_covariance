// -*- c++-mode -*-
#ifndef COVARIANCE_DISPLAY_H
#define COVARIANCE_DISPLAY_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

    class PoseWithCovarianceDisplay: public rviz::MessageFilterDisplay<geometry_msgs::PoseWithCovarianceStamped>
    {
        Q_OBJECT
        public:
            PoseWithCovarianceDisplay();
            virtual ~PoseWithCovarianceDisplay();

        protected:
            virtual void onInitialize();
            virtual void reset();

        private Q_SLOTS:
            void updateColorAndAlphaAndScale();

        private:
            void processMessage(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

            boost::shared_ptr<PoseWithCovarianceVisual> visual_;

            rviz::ColorProperty* color_property_;
            rviz::FloatProperty* alpha_property_;
            rviz::FloatProperty* scale_property_;
    };
} // end namespace rviz_plugin_covariance

#endif // COVARIANCE_DISPLAY_H
