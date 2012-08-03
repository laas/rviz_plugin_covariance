// -*- c++-mode -*-
#ifndef COVARIANCE_DISPLAY_H
# define COVARIANCE_DISPLAY_H
# include <message_filters/subscriber.h>
# include <tf/message_filter.h>
# include <sensor_msgs/Imu.h>
# include <rviz/display.h>

namespace Ogre
{
  class SceneNode;
}

namespace rviz_plugin_covariance
{
  class CovarianceVisual;

  class CovarianceDisplay: public rviz::Display
    {
    public:
      explicit CovarianceDisplay ();
      virtual ~CovarianceDisplay ();

      virtual void onInitialize ();
      virtual void fixedFrameChanged ();
      virtual void reset ();
      virtual void createProperties ();

      void setTopic (const std::string& topic);

      const std::string& getTopic () const
      {
	return topic_;
      }

      void setColor (const rviz::Color& color);

      const rviz::Color& getColor () const
      {
	return color_;
      }

      void setAlpha (float alpha);

      float getAlpha ()
      {
	return alpha_;
      }

      void setScale (float scale);

      float getScale ()
      {
	return scale_;
      }

    protected:
      virtual void onEnable ();
      virtual void onDisable ();

    private:
      void incomingMessage
	(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

      void subscribe ();
      void unsubscribe ();

      void clear ();

      void updateColorAndAlphaAndScale ();

      CovarianceVisual* visual_;

      Ogre::SceneNode* scene_node_;

      message_filters::Subscriber<
	geometry_msgs::PoseWithCovarianceStamped> sub_;
      tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>*
	tf_filter_;
      int messages_received_;

      rviz::Color color_;
      std::string topic_;
      float alpha_;
      float scale_;

      rviz::ColorPropertyWPtr color_property_;
      rviz::ROSTopicStringPropertyWPtr topic_property_;
      rviz::FloatPropertyWPtr alpha_property_;
      rviz::FloatPropertyWPtr scale_property_;
    };
} // end namespace rviz_plugin_covariance

#endif // COVARIANCE_DISPLAY_H
