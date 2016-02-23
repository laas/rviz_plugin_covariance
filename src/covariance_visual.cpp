#include "covariance_visual.h"

#include <rviz/ogre_helpers/shape.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <ros/console.h>

#include <sstream>

#include <Eigen/Dense>

using namespace rviz;

namespace rviz_plugin_covariance
{

CovarianceVisual::CovarianceVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, bool is_visible, float pos_scale, float ori_scale)
: Object( scene_manager ),
  position_scale_factor_( 1.0f ), orientation_scale_factor_( 1.0f ),
  position_msg_scale_(new Ogre::Vector3(0.0f,0.0f,0.0f)),
  orientation_msg_scale_(new Ogre::Vector3(0.0f,0.0f,0.0f))
{
  position_node_ = parent_node->createChildSceneNode();
  position_shape_ = new rviz::Shape(rviz::Shape::Sphere, scene_manager_, position_node_);

  orientation_node_ = parent_node->createChildSceneNode();
  orientation_shape_ = new rviz::Shape(rviz::Shape::Cone, scene_manager_, orientation_node_);

  setVisible( is_visible );

  setScales( pos_scale, ori_scale );
}

CovarianceVisual::~CovarianceVisual()
{
  delete position_shape_;
  delete orientation_shape_;

  scene_manager_->destroySceneNode( position_node_->getName() );
  scene_manager_->destroySceneNode( orientation_node_->getName() );
}

// This method compute the eigenvalues and eigenvectors of the position and orientation part covariance matrix
// separatelly and use their values to rotate and scale the covarance shapes:
// 
// - The largest scale will be the x-axis of the shape; the second largest the y-axis and the smallest the z-axis.
// 
// - The scaling of each axis will be defined by the eigenvalues: largest eigenvalue on x-axis, second on y-axis 
//   and smallest on z-axis.
// 
// - The rotation matrix is composed by the eigenvectors as columns, ordered in a decreasing order according to 
//   the respective eigenvalues.
// 
// - The rotation will make the x-axis of the shape coincide with the eigenvector of the largest eigenvalue, the 
//   y-axis coincide with the eigenvector of the second largest eigenvalue, and the z-axis coincide with the 
//   remaining eigenvector.
void CovarianceVisual::setCovariance( CovarianceVisual::covariance_type msg_covariance)
{
  // check for NaN in covariance
  for (unsigned i = 0; i < 3; ++i)
  {
      if(isnan(msg_covariance[i]))
      {
          ROS_WARN_THROTTLE(1, "covariance contains NaN");
          return;
      }
  }

  Eigen::Map< Eigen::Matrix<double,6,6> > covariance(msg_covariance.c_array());
  Eigen::Vector3d eigenvalues = Eigen::Vector3d::Identity();
  Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Zero();

  // NOTE: The SelfAdjointEigenSolver only references the lower triangular part of the covariance matrix
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance.topLeftCorner<3,3>());
  // Compute eigenvectors and eigenvalues
  if (eigensolver.info () == Eigen::Success)
  {
    eigenvalues = eigensolver.eigenvalues();   // can also be seen as variances
    eigenvectors = eigensolver.eigenvectors(); // ordered as column vectors
  }
  else
  {
    ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values for position. Is the covariance matrix correct?");
    eigenvalues = Eigen::Vector3d::Zero();      // This will set the scale to zero, hiding the covariance in the screen
    eigenvectors = Eigen::Matrix3d::Identity();
  }
  // Define the rotation and scale
  // NOTE: The solver return the eigenvalues and eigenvectors in a INCREASING order, thus we make some 
  //       changing in the orders here.
  Ogre::Quaternion positionQuaternion(Ogre::Matrix3(eigenvectors(0,2), eigenvectors(0,1), eigenvectors(0,0),
                                                    eigenvectors(1,2), eigenvectors(1,1), eigenvectors(1,0),
                                                    eigenvectors(2,2), eigenvectors(2,1), eigenvectors(2,0)));
  // The eigenvalue is the variance, so we take the sqrt to draw the standard deviation
  position_msg_scale_->x = std::sqrt (eigenvalues[2]);
  position_msg_scale_->y = std::sqrt (eigenvalues[1]);
  position_msg_scale_->z = std::sqrt (eigenvalues[0]);
  // The scale is also multiplied by a factor which should be set from outside (normally from a property)
  Ogre::Vector3 positionScaling = (*position_msg_scale_) * position_scale_factor_;

  // Repeat the same procedure for the orientation
  eigensolver.compute(covariance.bottomRightCorner<3,3>());
  if (eigensolver.info () == Eigen::Success)
  {
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors = eigensolver.eigenvectors();
  }
  else
  {
    ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values for position. Is the covariance matrix correct?");
    eigenvalues = Eigen::Vector3d::Zero();
    eigenvectors = Eigen::Matrix3d::Identity();
  }
  Ogre::Quaternion orientationQuaternion(Ogre::Matrix3(eigenvectors(0,2), eigenvectors(0,1), eigenvectors(0,0),
                                                       eigenvectors(1,2), eigenvectors(1,1), eigenvectors(1,0),
                                                       eigenvectors(2,2), eigenvectors(2,1), eigenvectors(2,0)));
  orientation_msg_scale_->x = std::sqrt (eigenvalues[2]);
  orientation_msg_scale_->y = std::sqrt (eigenvalues[1]);
  orientation_msg_scale_->z = std::sqrt (eigenvalues[0]);
  Ogre::Vector3 orientationScaling = (*orientation_msg_scale_) * orientation_scale_factor_;

  // Finnaly rotate and scale the nodes
  position_node_->setOrientation(positionQuaternion);
  orientation_node_->setOrientation(orientationQuaternion);

  if(!positionScaling.isNaN())
      position_node_->setScale(positionScaling);
  else
      ROS_WARN_STREAM("positionScaling contains NaN: " << positionScaling);

  if(!orientationScaling.isNaN())
      orientation_node_->setScale(orientationScaling);
  else
      ROS_WARN_STREAM("orientationScaling contains NaN: " << orientationScaling);
}

void CovarianceVisual::setScales( float pos_scale, float ori_scale)
{
  setPositionScale(pos_scale);
  setOrientationScale(ori_scale);
}

void CovarianceVisual::setPositionScale( float pos_scale ) 
{
  position_scale_factor_ = pos_scale;
  position_node_->setScale((*position_msg_scale_) * position_scale_factor_);
}

void CovarianceVisual::setOrientationScale( float ori_scale )
{
  orientation_scale_factor_ = ori_scale;
  orientation_node_->setScale((*orientation_msg_scale_) * orientation_scale_factor_);
}

void CovarianceVisual::setPositionColor(const Ogre::ColourValue& c)
{
  position_shape_->setColor(c);
}

void CovarianceVisual::setOrientationColor(const Ogre::ColourValue& c)
{
  orientation_shape_->setColor(c);
}

void CovarianceVisual::setPositionColor( float r, float g, float b, float a )
{
  setPositionColor( Ogre::ColourValue(r, g, b, a ));
}

void CovarianceVisual::setOrientationColor( float r, float g, float b, float a )
{
  setOrientationColor( Ogre::ColourValue(r, g, b, a ));
}

const Ogre::Vector3& CovarianceVisual::getPositionCovarianceScale()
{
  return position_node_->getScale();
}

const Ogre::Quaternion& CovarianceVisual::getPositionCovarianceOrientation()
{
  return position_node_->getOrientation();
}

const Ogre::Vector3& CovarianceVisual::getOrientationCovarianceScale()
{
  return orientation_node_->getScale();
}

const Ogre::Quaternion& CovarianceVisual::getOrientationCovarianceOrientation()
{
  return orientation_node_->getOrientation();
}

void CovarianceVisual::setUserData( const Ogre::Any& data )
{
  position_shape_->setUserData( data );
  orientation_shape_->setUserData( data );
}

void CovarianceVisual::setVisible( bool visible )
{
  position_node_->setVisible( visible );
  orientation_node_->setVisible( visible );
}

const Ogre::Vector3& CovarianceVisual::getPosition() 
{
  return position_node_->getPosition();
}

const Ogre::Quaternion& CovarianceVisual::getOrientation()
{
  return position_node_->getOrientation();
}

} // namespace rviz

