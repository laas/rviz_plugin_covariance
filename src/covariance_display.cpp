#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/properties/property_manager.h>
#include <rviz/frame_manager.h>

#include "covariance_visual.h"

#include "covariance_display.h"

namespace rviz_plugin_covariance
{
  CovarianceDisplay::CovarianceDisplay ()
    : Display (),
      visual_ (0),
      scene_node_ (0),
      messages_received_ (0),
      color_ (.8, .2, .8),
      alpha_ (1.0)
  {}

  void CovarianceDisplay::onInitialize()
  {
    scene_node_ =
      scene_manager_->getRootSceneNode ()->createChildSceneNode ();

    tf_filter_ =
      new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>
      (*vis_manager_->getTFClient (), "", 100, update_nh_);
    tf_filter_->connectInput (sub_);
    tf_filter_->registerCallback
      (boost::bind(&CovarianceDisplay::incomingMessage, this, _1));

    vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck
      (tf_filter_, this);
  }

  CovarianceDisplay::~CovarianceDisplay ()
  {
    unsubscribe ();
    clear ();
    delete visual_;
    delete tf_filter_;
  }

  // Clear the visuals by deleting their objects.
  void CovarianceDisplay::clear()
  {
    delete visual_;
    visual_ = 0;
    tf_filter_->clear ();
    messages_received_ = 0;
    setStatus (rviz::status_levels::Warn,
	       "Topic", "No messages received");
  }

  void CovarianceDisplay::setTopic (const std::string& topic)
  {
    unsubscribe ();
    clear ();
    topic_ = topic;
    subscribe ();

    // Broadcast the fact that the variable has changed.
    propertyChanged (topic_property_);

    // Make sure rviz renders the next time it gets a chance.
    causeRender ();
  }

  void CovarianceDisplay::setColor (const rviz::Color& color)
  {
    color_ = color;

    propertyChanged( color_property_ );
    updateColorAndAlpha();
    causeRender();
  }

  void CovarianceDisplay::setAlpha( float alpha )
  {
    alpha_ = alpha;

    propertyChanged( alpha_property_ );
    updateColorAndAlpha();
    causeRender();
  }

  // Set the current color and alpha values for each visual.
  void CovarianceDisplay::updateColorAndAlpha()
  {
    if (visual_)
      visual_->setColor (color_.r_, color_.g_, color_.b_, alpha_);
  }

  void CovarianceDisplay::subscribe()
  {
    // If we are not actually enabled, don't do it.
    if ( !isEnabled() )
      {
	return;
      }

    // Try to subscribe to the current topic name (in ``topic_``).  Make
    // sure to catch exceptions and set the status to a descriptive
    // error message.
    try
      {
	sub_.subscribe( update_nh_, topic_, 10 );
	setStatus( rviz::status_levels::Ok, "Topic", "OK" );
      }
    catch( ros::Exception& e )
      {
	setStatus( rviz::status_levels::Error, "Topic",
		   std::string( "Error subscribing: " ) + e.what() );
      }
  }

  void CovarianceDisplay::unsubscribe()
  {
    sub_.unsubscribe();
  }

  void CovarianceDisplay::onEnable()
  {
    subscribe();
  }

  void CovarianceDisplay::onDisable()
  {
    unsubscribe();
    clear();
  }

  void CovarianceDisplay::fixedFrameChanged()
  {
    tf_filter_->setTargetFrame( fixed_frame_ );
    clear();
  }

  void
  CovarianceDisplay::incomingMessage
  (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    ++messages_received_;

    std::stringstream ss;
    ss << messages_received_ << " messages received";
    setStatus( rviz::status_levels::Ok, "Topic", ss.str() );

    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if( !vis_manager_->getFrameManager()->getTransform
	(msg->header.frame_id,
	 msg->header.stamp,
	 position, orientation ))
      {
	ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
		   msg->header.frame_id.c_str(), fixed_frame_.c_str() );
	return;
      }

    CovarianceVisual* visual = visual_;
    if (visual == NULL)
      {
	visual = new CovarianceVisual
	  (vis_manager_->getSceneManager (), scene_node_);
	visual_ = visual;
      }

    visual->setMessage (msg);
    visual->setFramePosition (position);
    visual->setFrameOrientation (orientation);
    visual->setColor (color_.r_, color_.g_, color_.b_, alpha_);
  }

  void CovarianceDisplay::reset ()
  {
    Display::reset();
    clear();
  }

  void CovarianceDisplay::createProperties ()
  {
    topic_property_ =
      property_manager_->createProperty<rviz::ROSTopicStringProperty>
      ("Topic",
       property_prefix_,
       boost::bind (&CovarianceDisplay::getTopic, this),
       boost::bind (&CovarianceDisplay::setTopic, this, _1),
       parent_category_,
       this );
    setPropertyHelpText
      (topic_property_,
       "geometry_msgs::PoseWithCovarianceStamped topic to subscribe to.");
    rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
    topic_prop->setMessageType
      (ros::message_traits::datatype
       <geometry_msgs::PoseWithCovarianceStamped>());

    color_property_ =
      property_manager_->createProperty<rviz::ColorProperty>
      ("Color",
       property_prefix_,
       boost::bind( &CovarianceDisplay::getColor, this ),
       boost::bind( &CovarianceDisplay::setColor, this, _1 ),
       parent_category_,
       this);
    setPropertyHelpText
      (color_property_, "Color to draw the acceleration arrows.");

    alpha_property_ =
      property_manager_->createProperty<rviz::FloatProperty>
      ("Alpha",
       property_prefix_,
       boost::bind( &CovarianceDisplay::getAlpha, this ),
       boost::bind( &CovarianceDisplay::setAlpha, this, _1 ),
       parent_category_,
       this);
    setPropertyHelpText
      (alpha_property_, "0 is fully transparent, 1.0 is fully opaque.");
  }

} // end namespace rviz_plugin_covariance

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (rviz_plugin_covariance,
			 Covariance,
			 rviz_plugin_covariance::CovarianceDisplay,
			 rviz::Display)
