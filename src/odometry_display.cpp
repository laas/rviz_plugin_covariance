
#include <boost/bind.hpp>

#include <tf/transform_listener.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "odometry_display.h"
#include "covariance_property.h"
#include "covariance_visual.h"

using namespace rviz;

namespace rviz_plugin_covariance
{

OdometryDisplay::OdometryDisplay()
  : Display()
  , messages_received_(0)
{
  topic_property_ = new RosTopicProperty( "Topic", "",
                                          QString::fromStdString( ros::message_traits::datatype<nav_msgs::Odometry>() ),
                                          "nav_msgs::Odometry topic to subscribe to.",
                                          this, SLOT( updateTopic() ));

  color_property_ = new ColorProperty( "Color", QColor( 255, 25, 0 ),
                                       "Color of the arrows.",
                                       this, SLOT( updateColor() ));

  position_tolerance_property_ = new FloatProperty( "Position Tolerance", .1,
                                                    "Distance, in meters from the last arrow dropped, "
                                                    "that will cause a new arrow to drop.",
                                                    this );
  position_tolerance_property_->setMin( 0 );
                                                
  angle_tolerance_property_ = new FloatProperty( "Angle Tolerance", .1,
                                                 "Angular distance from the last arrow dropped, "
                                                 "that will cause a new arrow to drop.",
                                                 this );
  angle_tolerance_property_->setMin( 0 );

  keep_property_ = new IntProperty( "Keep", 100,
                                    "Number of arrows to keep before removing the oldest.  0 means keep all of them.",
                                    this );
  keep_property_->setMin( 0 );

  shaft_length_property_ = new FloatProperty( "Shaft Length", 1, "Length of the each arrow's shaft, in meters.",
                                              this, SLOT( updateArrowsGeometry() ));

  // aleeper: default changed from 0.1 to match change in arrow.cpp
  shaft_radius_property_ = new FloatProperty( "Shaft Radius", 0.05, "Radius of the each arrow's shaft, in meters.",
                                              this, SLOT( updateArrowsGeometry() ));
  
  head_length_property_ = new FloatProperty( "Head Length", 0.3, "Length of the each arrow's head, in meters.",
                                             this, SLOT( updateArrowsGeometry() ));

  // aleeper: default changed from 0.2 to match change in arrow.cpp
  head_radius_property_ = new FloatProperty( "Head Radius", 0.1, "Radius of the each arrow's head, in meters.",
                                             this, SLOT( updateArrowsGeometry() ));

  covariance_property_ = new CovarianceProperty( "Covariance", true, "Whether or not the covariances of the messages should be shown.",
                                             this, SLOT( updateCovarianceChoice() ));
 
  connect(covariance_property_, SIGNAL( childrenChanged() ), this, SLOT( updateCovarianceColorAndAlphaAndScale() ));

}

OdometryDisplay::~OdometryDisplay()
{
  if ( initialized() )
  {
    unsubscribe();
    clear();
    delete tf_filter_;
  }
}

void OdometryDisplay::onInitialize()
{
  tf_filter_ = new tf::MessageFilter<nav_msgs::Odometry>( *context_->getTFClient(), fixed_frame_.toStdString(),
                                                          5, update_nh_ );

  tf_filter_->connectInput( sub_ );
  tf_filter_->registerCallback( boost::bind( &OdometryDisplay::incomingMessage, this, _1 ));
  context_->getFrameManager()->registerFilterForTransformStatusCheck( tf_filter_, this );
}

void OdometryDisplay::clear()
{
  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    delete *it;
  }
  arrows_.clear();

  D_Covariance::iterator it_cov = covariances_.begin();
  D_Covariance::iterator end_cov = covariances_.end();
  for ( ; it_cov != end_cov; ++it_cov )
  {
    delete *it_cov;
  }
  covariances_.clear();

  if( last_used_message_ )
  {
    last_used_message_.reset();
  }

  tf_filter_->clear();

  messages_received_ = 0;
  setStatus( StatusProperty::Warn, "Topic", "No messages received" );
}

void OdometryDisplay::updateTopic()
{
  unsubscribe();
  clear();
  subscribe();
  context_->queueRender();
}

void OdometryDisplay::updateColor()
{
  QColor color = color_property_->getColor();
  float red   = color.redF();
  float green = color.greenF();
  float blue  = color.blueF();

  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for( ; it != end; ++it )
  {
    Arrow* arrow = *it;
    arrow->setColor( red, green, blue, 1.0f );
  }
  context_->queueRender();
}

void OdometryDisplay::updateArrowsGeometry()
{
  D_Arrow::iterator it = arrows_.begin();
  D_Arrow::iterator end = arrows_.end();
  for ( ; it != end; ++it )
  {
    updateGeometry(*it);
  }
  context_->queueRender();
}

void OdometryDisplay::updateGeometry( Arrow* arrow )
{
    arrow->set( shaft_length_property_->getFloat(),
                shaft_radius_property_->getFloat(),
                head_length_property_->getFloat(),
                head_radius_property_->getFloat() );
}

void OdometryDisplay::updateCovarianceChoice()
{
  updateCovarianceVisibility();
  context_->queueRender();
}

void OdometryDisplay::updateCovarianceVisibility()
{
  bool show_covariance = covariance_property_->getBool();

  D_Covariance::iterator it_cov = covariances_.begin();
  D_Covariance::iterator end_cov = covariances_.end();
  for ( ; it_cov != end_cov; ++it_cov )
  {
    CovarianceVisual* cov = *it_cov;
    cov->setVisible( show_covariance );
  }
}

void OdometryDisplay::updateCovarianceColorAndAlphaAndScale()
{
  QColor pos_color = covariance_property_->getPositionColor();
  float pos_alpha = covariance_property_->getPositionAlpha();
  float pos_scale = covariance_property_->getPositionScale();

  QColor ori_color = covariance_property_->getOrientationColor();
  float ori_alpha = covariance_property_->getOrientationAlpha();
  float ori_scale = covariance_property_->getOrientationScale();

  D_Covariance::iterator it_cov = covariances_.begin();
  D_Covariance::iterator end_cov = covariances_.end();
  for ( ; it_cov != end_cov; ++it_cov )
  {
    CovarianceVisual* cov = *it_cov;

    cov->setPositionColor( pos_color.redF(), pos_color.greenF(), pos_color.blueF(), pos_alpha );
    cov->setPositionScale( pos_scale );

    cov->setOrientationColor( ori_color.redF(), ori_color.greenF(), ori_color.blueF(), ori_alpha );
    cov->setOrientationScale( ori_scale );
  }

  context_->queueRender();
}

void OdometryDisplay::subscribe()
{
  if ( !isEnabled() )
  {
    return;
  }

  try
  {
    sub_.subscribe( update_nh_, topic_property_->getTopicStd(), 5 );
    setStatus( StatusProperty::Ok, "Topic", "OK" );
  }
  catch( ros::Exception& e )
  {
    setStatus( StatusProperty::Error, "Topic", QString( "Error subscribing: " ) + e.what() );
  }
}

void OdometryDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void OdometryDisplay::onEnable()
{
  subscribe();
}

void OdometryDisplay::onDisable()
{
  unsubscribe();
  clear();
}

bool validateFloats(const nav_msgs::Odometry& msg)
{
  bool valid = true;
  valid = valid && rviz::validateFloats( msg.pose.pose );
  valid = valid && rviz::validateFloats( msg.pose.covariance );
  valid = valid && rviz::validateFloats( msg.twist.twist );
  // valid = valid && rviz::validateFloats( msg.twist.covariance )
  return valid;
}

void OdometryDisplay::incomingMessage( const nav_msgs::Odometry::ConstPtr& message )
{
  ++messages_received_;

  if( !validateFloats( *message ))
  {
    setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  setStatus( StatusProperty::Ok, "Topic", QString::number( messages_received_ ) + " messages received" );

  if( last_used_message_ )
  {
    Ogre::Vector3 last_position(last_used_message_->pose.pose.position.x, last_used_message_->pose.pose.position.y, last_used_message_->pose.pose.position.z);
    Ogre::Vector3 current_position(message->pose.pose.position.x, message->pose.pose.position.y, message->pose.pose.position.z);
    Ogre::Quaternion last_orientation(last_used_message_->pose.pose.orientation.w, last_used_message_->pose.pose.orientation.x, last_used_message_->pose.pose.orientation.y, last_used_message_->pose.pose.orientation.z);
    Ogre::Quaternion current_orientation(message->pose.pose.orientation.w, message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z);

    if( (last_position - current_position).length() < position_tolerance_property_->getFloat() &&
        (last_orientation - current_orientation).normalise() < angle_tolerance_property_->getFloat() )
    {
      return;
    }
  }

  Arrow* arrow = new Arrow( scene_manager_, scene_node_, 0.8f, 0.05f, 0.2f, 0.2f );
  CovarianceVisual* cov = new CovarianceVisual( scene_manager_, arrow->getSceneNode() );

  transformArrow( message, arrow );

  QColor color = color_property_->getColor();
  arrow->setColor( color.redF(), color.greenF(), color.blueF(), 1.0f );

  updateGeometry(arrow);

  color = covariance_property_->getPositionColor();
  float alpha = covariance_property_->getPositionAlpha();
  cov->setPositionColor(color.redF(), color.greenF(), color.blueF(), alpha);

  color = covariance_property_->getOrientationColor();
  alpha = covariance_property_->getOrientationAlpha();
  cov->setOrientationColor(color.redF(), color.greenF(), color.blueF(), alpha);

  cov->setPositionScale( covariance_property_->getPositionScale() );
  cov->setOrientationScale( covariance_property_->getOrientationScale() );

  cov->setCovariance(message->pose.covariance);

  cov->setVisible( covariance_property_->getBool() );

  arrows_.push_back( arrow );
  covariances_.push_back( cov );

  last_used_message_ = message;
  context_->queueRender();
}

void OdometryDisplay::transformArrow( const nav_msgs::Odometry::ConstPtr& message, Arrow* arrow )
{
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->transform( message->header, message->pose.pose, position, orientation ))
  {
    ROS_ERROR( "Error transforming odometry '%s' from frame '%s' to frame '%s'",
               qPrintable( getName() ), message->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
  }

  arrow->setPosition( position );

  // Arrow points in -Z direction, so rotate the orientation before display.
  // TODO: is it safe to change Arrow to point in +X direction?
  arrow->setOrientation( orientation * Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Y ));
}

void OdometryDisplay::fixedFrameChanged()
{
  tf_filter_->setTargetFrame( fixed_frame_.toStdString() );
  clear();
}

void OdometryDisplay::update( float wall_dt, float ros_dt )
{
  size_t keep = keep_property_->getInt();
  if( keep > 0 )
  {
    while( arrows_.size() > keep )
    {
      delete arrows_.front();
      arrows_.pop_front();

      delete covariances_.front();
      covariances_.pop_front();
    }
  }

  assert(arrows_.size() == covariances_.size());
}

void OdometryDisplay::reset()
{
  Display::reset();
  clear();
}

void OdometryDisplay::setTopic( const QString &topic, const QString &datatype )
{
  topic_property_->setString( topic );
}

} // namespace rviz_plugin_covariance

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_plugin_covariance::OdometryDisplay, rviz::Display )
