#ifndef COVARIANCE_PROPERTY_H
#define COVARIANCE_PROPERTY_H

#include <QColor>

#include <OgreColourValue.h>

#include <rviz/properties/bool_property.h>

namespace rviz
{
class Property;
class ColorProperty;
class FloatProperty;
class EnumProperty;
}

namespace rviz_plugin_covariance
{

class CovarianceVisual;

/** @brief Property specialized to provide getter for booleans. */
class CovarianceProperty: public rviz::BoolProperty
{
Q_OBJECT
public:
  typedef boost::shared_ptr<CovarianceVisual> CovarianceVisualPtr;

  enum Frame
  {
    Rotating,
    Static,
  };

  CovarianceProperty( const QString& name = "Covariance",
                bool default_value = false,
                const QString& description = QString(),
                rviz::Property* parent = 0,
                const char *changed_slot = 0,
                QObject* receiver = 0 );

  virtual ~CovarianceProperty();

  // Methods to manage the deque of Covariance Visuals
  void pushBackVisual( const CovarianceVisualPtr& visual );
  void popFrontVisual();
  void clearVisual();
  size_t sizeVisual();

private Q_SLOTS:
  void updateColorAndAlphaAndScale();
  void updateOrientationFrame();
  void updateVisibility();

private:
  void updateColorAndAlphaAndScale( const CovarianceVisualPtr& visual );
  void updateOrientationFrame( const CovarianceVisualPtr& visual );
  void updateVisibility( const CovarianceVisualPtr& visual );

  typedef std::deque<CovarianceVisualPtr> D_Covariance;
  D_Covariance covariances_;

  rviz::BoolProperty*  position_property_;
  rviz::ColorProperty* position_color_property_;
  rviz::FloatProperty* position_alpha_property_;
  rviz::FloatProperty* position_scale_property_;
  rviz::BoolProperty*  orientation_property_;
  rviz::EnumProperty*  orientation_frame_property_;
  rviz::ColorProperty* orientation_color_property_;
  rviz::FloatProperty* orientation_alpha_property_;
  rviz::FloatProperty* orientation_scale_property_;
};

} // end namespace rviz_plugin_covariance

#endif // COVARIANCE_PROPERTY_H
