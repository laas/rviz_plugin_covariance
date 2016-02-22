#ifndef ODOMETRY_DISPLAY_H_
#define ODOMETRY_DISPLAY_H_

#include <deque>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#endif

#include <nav_msgs/Odometry.h>

#include "rviz/display.h"

namespace rviz
{

class Arrow;
class ColorProperty;
class FloatProperty;
class IntProperty;
class RosTopicProperty;
}

namespace rviz_plugin_covariance
{

class CovarianceVisual;
class CovarianceProperty;

/**
 * \class OdometryDisplay
 * \brief Accumulates and displays the pose from a nav_msgs::Odometry message
 */
class OdometryDisplay: public rviz::Display
{
Q_OBJECT
public:
  OdometryDisplay();
  virtual ~OdometryDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void update( float wall_dt, float ros_dt );
  virtual void reset();

  virtual void setTopic( const QString &topic, const QString &datatype );

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

private Q_SLOTS:
  void updateColor();
  void updateTopic();
  void updateArrowsGeometry();
  void updateCovarianceChoice();
  void updateCovarianceVisibility();
  void updateCovarianceColorAndAlphaAndScale();

private:
  void updateGeometry( rviz::Arrow* arrow );

  void subscribe();
  void unsubscribe();
  void clear();

  void incomingMessage( const nav_msgs::Odometry::ConstPtr& message );
  void transformArrow( const nav_msgs::Odometry::ConstPtr& message, rviz::Arrow* arrow );

  typedef std::deque<rviz::Arrow*> D_Arrow;
  typedef std::deque<CovarianceVisual*> D_Covariance;
  D_Arrow arrows_;
  D_Covariance covariances_;

  uint32_t messages_received_;

  nav_msgs::Odometry::ConstPtr last_used_message_;
  message_filters::Subscriber<nav_msgs::Odometry> sub_;
  tf::MessageFilter<nav_msgs::Odometry>* tf_filter_;

  rviz::ColorProperty* color_property_;
  rviz::RosTopicProperty* topic_property_;
  rviz::FloatProperty* position_tolerance_property_;
  rviz::FloatProperty* angle_tolerance_property_;
  rviz::IntProperty* keep_property_;
  
  rviz::FloatProperty* head_radius_property_;
  rviz::FloatProperty* head_length_property_;
  rviz::FloatProperty* shaft_radius_property_;
  rviz::FloatProperty* shaft_length_property_;

  CovarianceProperty* covariance_property_;
};

} // namespace rviz_plugin_covariance

#endif /* RVIZ_ODOMETRY_DISPLAY_H_ */
