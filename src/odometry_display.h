#ifndef ODOMETRY_DISPLAY_H_
#define ODOMETRY_DISPLAY_H_

#include <deque>

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#ifndef Q_MOC_RUN
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#endif

#include <rviz/message_filter_display.h>
#include <nav_msgs/Odometry.h>

namespace rviz
{
class Arrow;
class ColorProperty;
class FloatProperty;
class IntProperty;
}

namespace rviz_plugin_covariance
{

class CovarianceVisual;
class CovarianceProperty;

/**
 * \class OdometryDisplay
 * \brief Accumulates and displays the pose from a nav_msgs::Odometry message
 */
class OdometryDisplay: public rviz::MessageFilterDisplay<nav_msgs::Odometry>
{
Q_OBJECT
public:
  OdometryDisplay();
  virtual ~OdometryDisplay();

  // Overides MessageFilterDisplay
  virtual void reset();
  // Overides Display
  virtual void update( float wall_dt, float ros_dt );

private Q_SLOTS:
  void updateColor();
  void updateArrowsGeometry();
  void updateCovarianceChoice();
  void updateCovarianceVisibility();
  void updateCovarianceColorAndAlphaAndScale();

private:
  void updateGeometry( rviz::Arrow* arrow );
  void clear();

  virtual void processMessage( const nav_msgs::Odometry::ConstPtr& message );

  typedef std::deque<rviz::Arrow*> D_Arrow;
  typedef std::deque<CovarianceVisual*> D_Covariance;
  typedef std::deque<Ogre::SceneNode*> D_SceneNode;

  D_Arrow arrows_;
  D_Covariance covariances_;
  D_SceneNode scene_nodes_;

  nav_msgs::Odometry::ConstPtr last_used_message_;

  rviz::ColorProperty* color_property_;
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
