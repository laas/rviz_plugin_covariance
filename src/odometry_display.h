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
class Axes;
class ColorProperty;
class FloatProperty;
class IntProperty;
class EnumProperty;
}

namespace rviz_plugin_covariance
{

class CovarianceProperty;

/**
 * \class OdometryDisplay
 * \brief Accumulates and displays the pose from a nav_msgs::Odometry message
 */
class OdometryDisplay: public rviz::MessageFilterDisplay<nav_msgs::Odometry>
{
Q_OBJECT
public:
  enum Shape
  {
    ArrowShape,
    AxesShape,
  };

  OdometryDisplay();
  virtual ~OdometryDisplay();

  // Overides of MessageFilterDisplay
  virtual void onInitialize();
  virtual void reset();
  // Overides of Display
  virtual void update( float wall_dt, float ros_dt );

protected:
  /** @brief Overridden from MessageFilterDisplay to get Arrow/Axes visibility correct. */
  virtual void onEnable();

private Q_SLOTS:
  void updateShapeChoice();
  void updateShapeVisibility();
  void updateColorAndAlpha();
  void updateArrowsGeometry();
  void updateAxisGeometry();

private:
  void updateGeometry( rviz::Arrow* arrow );
  void updateGeometry( rviz::Axes* axes );
  void clear();

  virtual void processMessage( const nav_msgs::Odometry::ConstPtr& message );

  typedef std::deque<rviz::Arrow*> D_Arrow;
  typedef std::deque<rviz::Axes*> D_Axes;

  D_Arrow arrows_;
  D_Axes axes_;

  nav_msgs::Odometry::ConstPtr last_used_message_;

  rviz::EnumProperty* shape_property_;

  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::FloatProperty* position_tolerance_property_;
  rviz::FloatProperty* angle_tolerance_property_;
  rviz::IntProperty* keep_property_;
  
  rviz::FloatProperty* head_radius_property_;
  rviz::FloatProperty* head_length_property_;
  rviz::FloatProperty* shaft_radius_property_;
  rviz::FloatProperty* shaft_length_property_;

  rviz::FloatProperty* axes_length_property_;
  rviz::FloatProperty* axes_radius_property_;

  CovarianceProperty* covariance_property_;
};

} // namespace rviz_plugin_covariance

#endif /* RVIZ_ODOMETRY_DISPLAY_H_ */
