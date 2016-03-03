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

/** @brief Property specialized to provide getter for booleans. */
class CovarianceProperty: public rviz::BoolProperty
{
Q_OBJECT
public:
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

  Ogre::ColourValue getPositionOgreColor();
  QColor getPositionColor() const;
  float getPositionAlpha();
  float getPositionScale();
  virtual bool getPositionBool() const;

  Ogre::ColourValue getOrientationOgreColor();
  QColor getOrientationColor() const;
  float getOrientationAlpha();
  float getOrientationScale();
  virtual bool getOrientationBool() const;

  virtual int getOrientationFrameOptionInt() const;

Q_SIGNALS:
  bool childrenChanged();

private:
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
