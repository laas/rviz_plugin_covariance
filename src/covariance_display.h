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

  // Here we declare our new subclass of rviz::Display.  Every display
  // which can be listed in the "Displays" panel is a subclass of
  // rviz::Display.
  //
  // CovarianceDisplay will show a 3D arrow showing the direction and magnitude
  // of the IMU acceleration vector.  The base of the arrow will be at
  // the frame listed in the header of the Covariance message, and the
  // direction of the arrow will be relative to the orientation of that
  // frame.  It will also optionally show a history of recent
  // acceleration vectors, which will be stored in a circular buffer.
  //
  // The CovarianceDisplay class itself just implements the circular buffer,
  // editable parameters, and Display subclass machinery.  The visuals
  // themselves are represented by a separate class, CovarianceVisual.  The
  // idiom for the visuals is that when the objects exist, they appear
  // in the scene, and when they are deleted, they disappear.
  class CovarianceDisplay: public rviz::Display
    {
    public:
      // Constructor.  pluginlib::ClassLoader creates instances by calling
      // the default constructor, so make sure you have one.
      CovarianceDisplay();
      virtual ~CovarianceDisplay();

      // Overrides of public virtual functions from the Display class.
      virtual void onInitialize();
      virtual void fixedFrameChanged();
      virtual void reset();
      virtual void createProperties();

      // Setter and getter functions for user-editable properties.
      void setTopic(const std::string& topic);
      const std::string& getTopic() { return topic_; }

      void setColor( const rviz::Color& color );
      const rviz::Color& getColor() { return color_; }

      void setAlpha( float alpha );
      float getAlpha() { return alpha_; }

      // Overrides of protected virtual functions from Display.  As much
      // as possible, when Displays are not enabled, they should not be
      // subscribed to incoming data and should not show anything in the
      // 3D view.  These functions are where these connections are made
      // and broken.
    protected:
      virtual void onEnable();
      virtual void onDisable();

      // Function to handle an incoming ROS message.
    private:
      void incomingMessage
	(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

      // Internal helpers which do the work of subscribing and
      // unsubscribing from the ROS topic.
      void subscribe();
      void unsubscribe();

      // A helper to clear this display back to the initial state.
      void clear();

      // Helper function to apply color and alpha to all visuals.
      void updateColorAndAlpha();

      // Storage for the list of visuals.  This display supports an
      // adjustable history length, so we need one visual per history
      // item.
      CovarianceVisual* visual_;

      // A node in the Ogre scene tree to be the parent of all our visuals.
      Ogre::SceneNode* scene_node_;

      // Data input: Subscriber and tf message filter.
      message_filters::Subscriber<
	geometry_msgs::PoseWithCovarianceStamped> sub_;
      tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>*
	tf_filter_;
      int messages_received_;

      // User-editable property variables.
      rviz::Color color_;
      std::string topic_;
      float alpha_;
      int history_length_;

      // Property objects for user-editable properties.
      rviz::ColorPropertyWPtr color_property_;
      rviz::ROSTopicStringPropertyWPtr topic_property_;
      rviz::FloatPropertyWPtr alpha_property_;
    };
} // end namespace rviz_plugin_tutorials

#endif // COVARIANCE_DISPLAY_H
