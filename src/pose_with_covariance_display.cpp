/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OgreEntity.h>
#include <OgreSceneNode.h>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/validate_floats.h>

#include "pose_with_covariance_display.h"

#include <Eigen/Dense>

using namespace rviz;

namespace rviz_plugin_covariance
{

// Anonymous namespace with helper functions
namespace
{
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> computeEigenValuesAndVectors
                                                (const geometry_msgs::PoseWithCovariance& msg,
                                                 unsigned offset)
    {
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        Eigen::Vector3d eigenValues = Eigen::Vector3d::Identity();
        Eigen::Matrix3d eigenVectors = Eigen::Matrix3d::Zero();

        for(unsigned i = 0; i < 3; ++i)
            for(unsigned j = 0; j < 3; ++j)
                covariance(i, j) = msg.covariance[(i + offset) * 6 + j + offset];

        // Compute eigen values and eigen vectors.
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);

        if (eigensolver.info () == Eigen::Success)
        {
            eigenValues = eigensolver.eigenvalues();
            eigenVectors = eigensolver.eigenvectors();
        }
        else
            ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values. Is the covariance matrix correct?");

        return std::make_pair (eigenVectors, eigenValues);
    }

    Ogre::Quaternion computeRotation
                     (const geometry_msgs::PoseWithCovariance& msg,
                      std::pair<Eigen::Matrix3d, Eigen::Vector3d>& pair)
    {
        Ogre::Matrix3 rotation;

        for (unsigned i = 0; i < 3; ++i)
            for (unsigned j = 0; j < 3; ++j)
                rotation[i][j] = pair.first(i, j);


        if(std::abs(Ogre::Quaternion(rotation).Norm() - 1.0f) > 0.00001)
        {
            ROS_WARN("computeRotation found non-unitary quaternion!");

            // Check for swapped eigenvectors within repeated eigenvalues
            if(pair.second(0) == pair.second(1))
            {
                ROS_WARN("repeated eigen values 0 and 1... attempting swap");
                for (unsigned i = 0; i < 3; ++i)
                {
                    rotation[i][0] = pair.first(i, 1);
                    rotation[i][1] = pair.first(i, 0);
                }
            }
            else if(pair.second(1) == pair.second(2))
            {
                ROS_WARN("repeated eigen values 1 and 2... attempting swap");
                for (unsigned i = 0; i < 3; ++i)
                {
                    rotation[i][1] = pair.first(i, 2);
                    rotation[i][2] = pair.first(i, 1);
                }
            }
        }

        return Ogre::Quaternion(rotation);
    }
} // end of anonymous namespace.

class PoseWithCovarianceDisplaySelectionHandler: public SelectionHandler
{
public:
  PoseWithCovarianceDisplaySelectionHandler( PoseWithCovarianceDisplay* display, DisplayContext* context )
    : SelectionHandler( context )
    , display_( display )
  {}

  void createProperties( const Picked& obj, Property* parent_property )
  {
    Property* cat = new Property( "Pose " + display_->getName(), QVariant(), "", parent_property );
    properties_.push_back( cat );

    frame_property_ = new StringProperty( "Frame", "", "", cat );
    frame_property_->setReadOnly( true );

    position_property_ = new VectorProperty( "Position", Ogre::Vector3::ZERO, "", cat );
    position_property_->setReadOnly( true );

    orientation_property_ = new QuaternionProperty( "Orientation", Ogre::Quaternion::IDENTITY, "", cat );
    orientation_property_->setReadOnly( true );

    covariance_position_property_ = new VectorProperty( "Covariance Position", Ogre::Vector3::ZERO, "", cat );
    covariance_position_property_->setReadOnly( true );

    covariance_orientation_property_ = new VectorProperty( "Covariance Orientation", Ogre::Vector3::ZERO, "", cat );
    covariance_orientation_property_->setReadOnly( true );    
  }

  void getAABBs( const Picked& obj, V_AABB& aabbs )
  {
    if( display_->pose_valid_ )
    {
      if( display_->shape_property_->getOptionInt() == PoseWithCovarianceDisplay::Arrow )
      {
        aabbs.push_back( display_->arrow_->getHead()->getEntity()->getWorldBoundingBox() );
        aabbs.push_back( display_->arrow_->getShaft()->getEntity()->getWorldBoundingBox() );
      }
      else
      {
        aabbs.push_back( display_->axes_->getXShape()->getEntity()->getWorldBoundingBox() );
        aabbs.push_back( display_->axes_->getYShape()->getEntity()->getWorldBoundingBox() );
        aabbs.push_back( display_->axes_->getZShape()->getEntity()->getWorldBoundingBox() );
      }
    }

    if( display_->covariance_valid_ )
    {
      if( display_->covariance_property_->getBool() )
      {
        // NOTE: The bounding boxes looks correct when rviz is fixed in the pose, but looks weird when fixed on other frames.
        //       Not sure if it's wrong or it's ok. It appears to happen with topics with low publishing rate.
        aabbs.push_back( display_->covariance_position_shape_->getEntity()->getWorldBoundingBox() );
        aabbs.push_back( display_->covariance_orientation_shape_->getEntity()->getWorldBoundingBox() );
      }
    }
  }

  void setMessage(const geometry_msgs::PoseWithCovarianceStampedConstPtr& message)
  {
    // properties_.size() should only be > 0 after createProperties()
    // and before destroyProperties(), during which frame_property_,
    // position_property_, and orientation_property_ should be valid
    // pointers.
    if( properties_.size() > 0 )
    {
      frame_property_->setStdString( message->header.frame_id );
      position_property_->setVector( Ogre::Vector3( message->pose.pose.position.x,
                                                    message->pose.pose.position.y,
                                                    message->pose.pose.position.z ));
      orientation_property_->setQuaternion( Ogre::Quaternion( message->pose.pose.orientation.w,
                                                              message->pose.pose.orientation.x,
                                                              message->pose.pose.orientation.y,
                                                              message->pose.pose.orientation.z ));
      covariance_position_property_->setVector( Ogre::Vector3( message->pose.covariance[0+0*6],
                                                               message->pose.covariance[1+1*6],
                                                               message->pose.covariance[2+2*6] ));

      covariance_orientation_property_->setVector( Ogre::Vector3( message->pose.covariance[3+3*6],
                                                                  message->pose.covariance[4+4*6],
                                                                  message->pose.covariance[5+5*6] ));
    }
  }

private:
  PoseWithCovarianceDisplay* display_;
  StringProperty* frame_property_;
  VectorProperty* position_property_;
  QuaternionProperty* orientation_property_;
  VectorProperty* covariance_position_property_;
  VectorProperty* covariance_orientation_property_;
  
};

PoseWithCovarianceDisplay::PoseWithCovarianceDisplay()
  : pose_valid_( false ),
    covariance_valid_( false )
{
  shape_property_ = new EnumProperty( "Shape", "Arrow", "Shape to display the pose as.",
                                      this, SLOT( updateShapeChoice() ));
  shape_property_->addOption( "Arrow", Arrow );
  shape_property_->addOption( "Axes", Axes );

  color_property_ = new ColorProperty( "Color", QColor( 255, 25, 0 ), "Color to draw the arrow.",
                                       this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new FloatProperty( "Alpha", 1, "Amount of transparency to apply to the arrow.",
                                       this, SLOT( updateColorAndAlpha() ));
  alpha_property_->setMin( 0 );
  alpha_property_->setMax( 1 );

  shaft_length_property_ = new FloatProperty( "Shaft Length", 1, "Length of the arrow's shaft, in meters.",
                                              this, SLOT( updateArrowGeometry() ));

  // aleeper: default changed from 0.1 to match change in arrow.cpp
  shaft_radius_property_ = new FloatProperty( "Shaft Radius", 0.05, "Radius of the arrow's shaft, in meters.",
                                              this, SLOT( updateArrowGeometry() ));
  
  head_length_property_ = new FloatProperty( "Head Length", 0.3, "Length of the arrow's head, in meters.",
                                             this, SLOT( updateArrowGeometry() ));

  // aleeper: default changed from 0.2 to match change in arrow.cpp
  head_radius_property_ = new FloatProperty( "Head Radius", 0.1, "Radius of the arrow's head, in meters.",
                                             this, SLOT( updateArrowGeometry() ));

  axes_length_property_ = new FloatProperty( "Axes Length", 1, "Length of each axis, in meters.",
                                             this, SLOT( updateAxisGeometry() ));

  axes_radius_property_ = new FloatProperty( "Axes Radius", 0.1, "Radius of each axis, in meters.",
                                             this, SLOT( updateAxisGeometry() ));

  covariance_property_ = new BoolProperty( "Covariance", true, "Whether or not the covariances of the messages should be shown.",
                                             this, SLOT( updateCovarianceChoice() ));
  
  covariance_position_color_property_ = new ColorProperty( "Position Color", QColor( 204, 51, 204 ),
                                             "Color to draw the covariance ellipse.",
                                             covariance_property_, SLOT( updateCovarianceColorAndAlphaAndScale() ), 
                                             this);
  
  covariance_position_alpha_property_ = new FloatProperty( "Position Alpha", 0.3f,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             covariance_property_, SLOT( updateCovarianceColorAndAlphaAndScale() ),
                                             this);
  
  covariance_position_scale_property_ = new FloatProperty( "Position Scale", 1.0f,
                                             "Scale factor to be applied to covariance ellipse",
                                             covariance_property_, SLOT( updateCovarianceColorAndAlphaAndScale() ),
                                             this);

  covariance_orientation_color_property_ = new ColorProperty( "Orientation Color", QColor( 255, 255, 127 ),
                                             "Color to draw the covariance ellipse.",
                                             covariance_property_, SLOT( updateCovarianceColorAndAlphaAndScale() ), 
                                             this);
  
  covariance_orientation_alpha_property_ = new FloatProperty( "Orientation Alpha", 0.5f,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             covariance_property_, SLOT( updateCovarianceColorAndAlphaAndScale() ),
                                             this);
  
  covariance_orientation_scale_property_ = new FloatProperty( "Orientation Scale", 1.0f,
                                             "Scale factor to be applied to covariance ellipse",
                                             covariance_property_, SLOT( updateCovarianceColorAndAlphaAndScale() ),
                                             this);

}

void PoseWithCovarianceDisplay::onInitialize()
{
  MFDClass::onInitialize();

  arrow_ = new rviz::Arrow( scene_manager_, scene_node_,
                            shaft_length_property_->getFloat(),
                            shaft_radius_property_->getFloat(),
                            head_length_property_->getFloat(),
                            head_radius_property_->getFloat() );
  // Arrow points in -Z direction, so rotate the orientation before display.
  // TODO: is it safe to change Arrow to point in +X direction?
  arrow_->setOrientation( Ogre::Quaternion( Ogre::Degree( -90 ), Ogre::Vector3::UNIT_Y ));

  axes_ = new rviz::Axes( scene_manager_, scene_node_,
                          axes_length_property_->getFloat(),
                          axes_radius_property_->getFloat() );

  // covariance_position_node_ = scene_node_->createChildSceneNode();
  covariance_position_node_ = axes_->getSceneNode()->createChildSceneNode();
  covariance_position_shape_ = new rviz::Shape(rviz::Shape::Sphere, scene_manager_, covariance_position_node_);

  covariance_orientation_node_ = axes_->getSceneNode()->createChildSceneNode();
  covariance_orientation_shape_ = new rviz::Shape(rviz::Shape::Cone, scene_manager_, covariance_orientation_node_);

  updateShapeChoice();
  updateColorAndAlpha();
  updateCovarianceChoice();
  updateCovarianceColorAndAlphaAndScale();

  coll_handler_.reset( new PoseWithCovarianceDisplaySelectionHandler( this, context_ ));
  coll_handler_->addTrackedObjects( arrow_->getSceneNode() );
  coll_handler_->addTrackedObjects( axes_->getSceneNode() );
  // TODO: Not sure if getRootNode is the function to be called here...
  coll_handler_->addTrackedObjects( covariance_position_shape_->getRootNode() );
}

PoseWithCovarianceDisplay::~PoseWithCovarianceDisplay()
{
  if ( initialized() )
  {
    delete arrow_;
    delete axes_;
    delete covariance_position_shape_;
  }
}

void PoseWithCovarianceDisplay::onEnable()
{
  MFDClass::onEnable();
  updateShapeVisibility();
}

void PoseWithCovarianceDisplay::updateColorAndAlpha()
{
  Ogre::ColourValue color = color_property_->getOgreColor();
  color.a = alpha_property_->getFloat();

  arrow_->setColor( color );

  context_->queueRender();
}

void PoseWithCovarianceDisplay::updateCovarianceColorAndAlphaAndScale()
{
  Ogre::ColourValue color = covariance_position_color_property_->getOgreColor();
  color.a = covariance_position_alpha_property_->getFloat();

  covariance_position_shape_->setColor( color );

  covariance_position_scale_ = covariance_position_scale_property_->getFloat();

  color = covariance_orientation_color_property_->getOgreColor();
  color.a = covariance_orientation_alpha_property_->getFloat();

  covariance_orientation_shape_->setColor( color );

  covariance_orientation_scale_ = covariance_orientation_scale_property_->getFloat();

  context_->queueRender();
}

void PoseWithCovarianceDisplay::updateArrowGeometry()
{
  arrow_->set( shaft_length_property_->getFloat(),
               shaft_radius_property_->getFloat(),
               head_length_property_->getFloat(),
               head_radius_property_->getFloat() );
  context_->queueRender();
}

void PoseWithCovarianceDisplay::updateAxisGeometry()
{
  axes_->set( axes_length_property_->getFloat(),
              axes_radius_property_->getFloat() );
  context_->queueRender();
}

void PoseWithCovarianceDisplay::updateShapeChoice()
{
  bool use_arrow = ( shape_property_->getOptionInt() == Arrow );

  color_property_->setHidden( !use_arrow );
  alpha_property_->setHidden( !use_arrow );
  shaft_length_property_->setHidden( !use_arrow );
  shaft_radius_property_->setHidden( !use_arrow );
  head_length_property_->setHidden( !use_arrow );
  head_radius_property_->setHidden( !use_arrow );

  axes_length_property_->setHidden( use_arrow );
  axes_radius_property_->setHidden( use_arrow );

  updateShapeVisibility();

  context_->queueRender();
}

void PoseWithCovarianceDisplay::updateCovarianceChoice()
{
  bool show_covariance = covariance_property_->getBool() ;

  covariance_position_scale_property_->setHidden( !show_covariance );
  covariance_position_alpha_property_->setHidden( !show_covariance );
  covariance_position_color_property_->setHidden( !show_covariance );

  covariance_orientation_scale_property_->setHidden( !show_covariance );
  covariance_orientation_alpha_property_->setHidden( !show_covariance );
  covariance_orientation_color_property_->setHidden( !show_covariance );

  updateShapeVisibility();

  context_->queueRender();
}

void PoseWithCovarianceDisplay::updateShapeVisibility()
{
  if( !pose_valid_ )
  {
    arrow_->getSceneNode()->setVisible( false );
    axes_->getSceneNode()->setVisible( false );
  }
  else
  {
    bool use_arrow = (shape_property_->getOptionInt() == Arrow);
    arrow_->getSceneNode()->setVisible( use_arrow );
    axes_->getSceneNode()->setVisible( !use_arrow );
  }

  if( !covariance_valid_ )
  {
    // TODO: Not sure if getRootNode is the function to be called here...
    covariance_position_shape_->getRootNode()->setVisible( false );
    covariance_orientation_shape_->getRootNode()->setVisible( false );
  }
  else
  {
    bool show_covariance = covariance_property_->getBool();
    // TODO: Not sure if getRootNode is the function to be called here...
    covariance_position_shape_->getRootNode()->setVisible( show_covariance );
    covariance_orientation_shape_->getRootNode()->setVisible( show_covariance );
  }
}

void PoseWithCovarianceDisplay::processMessage( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message )
{
  if( !validateFloats( message->pose.pose ) || !validateFloats( message->pose.covariance ))
  {
    setStatus( StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
    return;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if( !context_->getFrameManager()->transform( message->header, message->pose.pose, position, orientation ))
  {
    ROS_ERROR( "Error transforming pose '%s' from frame '%s' to frame '%s'",
               qPrintable( getName() ), message->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }

  pose_valid_ = true;
  covariance_valid_ = true;
  updateShapeVisibility();

  scene_node_->setPosition( position );
  scene_node_->setOrientation( orientation );

  setCovarianceNodes( message->pose );

  coll_handler_->setMessage( message );

  context_->queueRender();
}

void PoseWithCovarianceDisplay::setCovarianceNodes(const geometry_msgs::PoseWithCovariance& msg)
{
    // Construct pose position and orientation.
    const geometry_msgs::Point& p = msg.pose.position;
    const geometry_msgs::Quaternion& q = msg.pose.orientation;

    Ogre::Vector3 position(p.x, p.y, p.z);
    Ogre::Quaternion orientation(q.w, q.x, q.y, q.z);

    // // Set position and orientation for axes scene node.
    // if(!position.isNaN())
    //     axes_->setPosition(position);
    // else
    //     ROS_WARN_STREAM_THROTTLE(1, "position contains NaN: " << position);

    // if(!orientation.isNaN())
    //     axes_->setOrientation (orientation);
    // else
    //     ROS_WARN_STREAM_THROTTLE(1, "orientation contains NaN: " << orientation);

    // check for NaN in covariance
    for (unsigned i = 0; i < 3; ++i)
    {
        if(isnan(msg.covariance[i]))
        {
            ROS_WARN_THROTTLE(1, "covariance contains NaN");
            return;
        }
    }

    // Compute eigen values and vectors for both shapes.
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> positionEigenVectorsAndValues(computeEigenValuesAndVectors(msg, 0));
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> orientationEigenVectorsAndValues(computeEigenValuesAndVectors(msg, 3));

    Ogre::Quaternion positionQuaternion(computeRotation(msg, positionEigenVectorsAndValues));
    Ogre::Quaternion orientationQuaternion(computeRotation(msg, orientationEigenVectorsAndValues));

    covariance_position_node_->setOrientation(positionQuaternion);
    covariance_orientation_node_->setOrientation(orientationQuaternion);

    // Compute scaling.
    //Ogre::Vector3 axesScaling(1, 1, 1);

    //axesScaling *= scaleFactor_;

    Ogre::Vector3 positionScaling
                  (std::sqrt (positionEigenVectorsAndValues.second[0]),
                   std::sqrt (positionEigenVectorsAndValues.second[1]),
                   std::sqrt (positionEigenVectorsAndValues.second[2]));

    positionScaling *= covariance_position_scale_;

    Ogre::Vector3 orientationScaling
                  (std::sqrt (orientationEigenVectorsAndValues.second[0]),
                   std::sqrt (orientationEigenVectorsAndValues.second[1]),
                   std::sqrt (orientationEigenVectorsAndValues.second[2]));

    orientationScaling *= covariance_orientation_scale_;

    // Set the scaling.
    /*if(!axesScaling.isNaN())
        axes_->setScale(axesScaling);
    else
        ROS_WARN_STREAM("axesScaling contains NaN: " << axesScaling);*/

    if(!positionScaling.isNaN())
        covariance_position_node_->setScale(positionScaling);
    else
        ROS_WARN_STREAM("positionScaling contains NaN: " << positionScaling);

    if(!orientationScaling.isNaN())
        covariance_orientation_node_->setScale(orientationScaling);
    else
        ROS_WARN_STREAM("orientationScaling contains NaN: " << orientationScaling);

    // Debugging.
    ROS_DEBUG_STREAM_THROTTLE
    (1.,
    "Position:\n"
    << position << "\n"
    << "Positional part 3x3 eigen values:\n"
    << positionEigenVectorsAndValues.second << "\n"
    << "Positional part 3x3 eigen vectors:\n"
    << positionEigenVectorsAndValues.first << "\n"
    << "Sphere orientation:\n"
    << positionQuaternion << "\n"
    << positionQuaternion.getRoll () << " "
    << positionQuaternion.getPitch () << " "
    << positionQuaternion.getYaw () << "\n"
    << "Sphere scaling:\n"
    << positionScaling << "\n"
    << "Rotational part 3x3 eigen values:\n"
    << orientationEigenVectorsAndValues.second << "\n"
    << "Rotational part 3x3 eigen vectors:\n"
    << orientationEigenVectorsAndValues.first << "\n"
    << "Cone orientation:\n"
    << orientationQuaternion << "\n"
    << orientationQuaternion.getRoll () << " "
    << orientationQuaternion.getPitch () << " "
    << orientationQuaternion.getYaw () << "\n"
    << "Cone scaling:\n"
    << orientationScaling
    );
}


void PoseWithCovarianceDisplay::reset()
{
  MFDClass::reset();
  pose_valid_ = false;
  covariance_valid_ = false;
  updateShapeVisibility();
}

} // namespace rviz_plugin_covariance

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_plugin_covariance::PoseWithCovarianceDisplay, rviz::Display )
