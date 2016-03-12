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

namespace Ogre
{
  class SceneManager;
  class SceneNode;
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
    Local,
    Fixed,
  };

  enum ColorStyle
  {
    Unique,
    RGB,
  };

  CovarianceProperty( const QString& name = "Covariance",
                bool default_value = false,
                const QString& description = QString(),
                rviz::Property* parent = 0,
                const char *changed_slot = 0,
                QObject* receiver = 0 );

  virtual ~CovarianceProperty();

  bool getPositionBool();
  bool getOrientationBool();

  // Methods to manage the deque of Covariance Visuals
  CovarianceVisualPtr createAndPushBackVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  void popFrontVisual();
  void clearVisual();
  size_t sizeVisual();

public Q_SLOTS:
  void updateVisibility();

private Q_SLOTS:
  void updateColorAndAlphaAndScaleAndOffset();
  void updateOrientationFrame();
  void updateColorStyleChoice();

private:
  void updateColorAndAlphaAndScaleAndOffset( const CovarianceVisualPtr& visual );
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
  rviz::EnumProperty*  orientation_colorstyle_property_;
  rviz::ColorProperty* orientation_color_property_;
  rviz::FloatProperty* orientation_alpha_property_;
  rviz::FloatProperty* orientation_offset_property_;
  rviz::FloatProperty* orientation_scale_property_;
};

} // end namespace rviz_plugin_covariance

#endif // COVARIANCE_PROPERTY_H
