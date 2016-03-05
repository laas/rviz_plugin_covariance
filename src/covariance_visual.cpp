#include "covariance_visual.h"

#include <rviz/ogre_helpers/shape.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

#include <ros/console.h>

#include <sstream>

using namespace rviz;

namespace rviz_plugin_covariance
{

double deg2rad (double degrees) {
    return degrees * 4.0 * atan (1.0) / 180.0;
}

CovarianceVisual::CovarianceVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, bool is_local_rotation, bool is_visible, float pos_scale, float ori_scale)
: Object( scene_manager ), local_rotation_(is_local_rotation)
{
  // Main node of the visual
  root_node_ = parent_node->createChildSceneNode();
  // Node that will have the same orientation as the fixed frame. Updated from the message on setCovariance()
  fixed_orientation_node_ = root_node_->createChildSceneNode();
  // Node to scale the position part of the covariance from the property value
  position_scale_node_ = fixed_orientation_node_->createChildSceneNode();
  // Node to be oriented and scaled from the message's covariance
  position_node_ = position_scale_node_->createChildSceneNode();
  position_shape_ = new rviz::Shape(rviz::Shape::Sphere, scene_manager_, position_node_);

  // Node to scale the orientation part of the covariance. May be attached to both the local (root) node or the fixed frame node.
  // May be re-attached later by setRotatingFrame()
  if(local_rotation_)
    orientation_scale_node_ = root_node_->createChildSceneNode();
  else
    orientation_scale_node_ = fixed_orientation_node_->createChildSceneNode();

  for(int i = 0; i < 3; i++)
  {
    // Node to position and orient the shape along the axis. One for each axis.
    orientation_offset_node_[i] = orientation_scale_node_->createChildSceneNode();
    // Does not inherit scale from the parent. This is needed to keep the cylinders with the same height. The scale is set by setOrientationScale()
    orientation_offset_node_[i]->setInheritScale( false );
    // Node to be oriented and scaled by the message's covariance. One for each axis.
    orientation_node_[i] = orientation_offset_node_[i]->createChildSceneNode();
    orientation_shape_[i] = new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, orientation_node_[i]);
  }

  // Position the cylindes at position 1.0 in the respective axis, and perpendicular to the axis.
  // x-axis (roll)
  orientation_offset_node_[kRoll]->setPosition( Ogre::Vector3::UNIT_X );
  orientation_offset_node_[kRoll]->setOrientation( Ogre::Quaternion(Ogre::Degree( 90 ), Ogre::Vector3::UNIT_X ) * Ogre::Quaternion( Ogre::Degree( 90 ), Ogre::Vector3::UNIT_Z ) );
  // y-axis (pitch)
  orientation_offset_node_[kPitch]->setPosition( Ogre::Vector3( Ogre::Vector3::UNIT_Y ) );
  orientation_offset_node_[kPitch]->setOrientation( Ogre::Quaternion( Ogre::Degree( 90 ), Ogre::Vector3::UNIT_Y ) );
  // z-axis (yaw)
  orientation_offset_node_[kYaw]->setPosition( Ogre::Vector3( Ogre::Vector3::UNIT_Z ) );
  orientation_offset_node_[kYaw]->setOrientation( Ogre::Quaternion( Ogre::Degree( 90 ), Ogre::Vector3::UNIT_X ) );

  // set initial visibility and scale
  root_node_->setVisible( is_visible );
  setScales( pos_scale, ori_scale );
}

CovarianceVisual::~CovarianceVisual()
{
  delete position_shape_;
  scene_manager_->destroySceneNode( position_node_->getName() );

  for(int i = 0; i < 3; i++)
  {
    delete orientation_shape_[i];
    scene_manager_->destroySceneNode( orientation_node_[i]->getName() );
    scene_manager_->destroySceneNode( orientation_offset_node_[i]->getName() );
  }

  scene_manager_->destroySceneNode( position_scale_node_->getName() );
  scene_manager_->destroySceneNode( fixed_orientation_node_->getName() );
  scene_manager_->destroySceneNode( root_node_->getName() );
}

// Local function to force the axis to be right handed for 3D. Taken from ecl_statistics
void makeRightHanded( Eigen::Matrix3d& eigenvectors, Eigen::Vector3d& eigenvalues)
{
  // Note that sorting of eigenvalues may end up with left-hand coordinate system.
  // So here we correctly sort it so that it does end up being righ-handed and normalised.
  Eigen::Vector3d c0 = eigenvectors.block<3,1>(0,0);  c0.normalize();
  Eigen::Vector3d c1 = eigenvectors.block<3,1>(0,1);  c1.normalize();
  Eigen::Vector3d c2 = eigenvectors.block<3,1>(0,2);  c2.normalize();
  Eigen::Vector3d cc = c0.cross(c1);
  if (cc.dot(c2) < 0) {
    eigenvectors << c1, c0, c2;
    double e = eigenvalues[0];  eigenvalues[0] = eigenvalues[1];  eigenvalues[1] = e;
  } else {
    eigenvectors << c0, c1, c2;
  }
}

// Local function to force the axis to be right handed for 2D. Based on the one from ecl_statistics
void makeRightHanded( Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues)
{
  // Note that sorting of eigenvalues may end up with left-hand coordinate system.
  // So here we correctly sort it so that it does end up being righ-handed and normalised.
  Eigen::Vector3d c0;  c0.setZero();  c0.head<2>() = eigenvectors.col(0);  c0.normalize();
  Eigen::Vector3d c1;  c1.setZero();  c1.head<2>() = eigenvectors.col(1);  c1.normalize();
  Eigen::Vector3d cc = c0.cross(c1);
  if (cc[2] < 0) {
    eigenvectors << c1.head<2>(), c0.head<2>();
    double e = eigenvalues[0];  eigenvalues[0] = eigenvalues[1];  eigenvalues[1] = e;
  } else {
    eigenvectors << c0.head<2>(), c1.head<2>();
  }
}

void computeShapeScaleAndOrientation3D(const Eigen::Matrix3d& covariance, Ogre::Vector3& scale, Ogre::Quaternion& orientation)
{
  Eigen::Vector3d eigenvalues(Eigen::Vector3d::Identity());
  Eigen::Matrix3d eigenvectors(Eigen::Matrix3d::Zero());

  // NOTE: The SelfAdjointEigenSolver only references the lower triangular part of the covariance matrix
  // FIXME: Should we use Eigen's pseudoEigenvectors() ?
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
  // Compute eigenvectors and eigenvalues
  if (eigensolver.info () == Eigen::Success)
  {
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors = eigensolver.eigenvectors();
  }
  else
  {
    ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values for position. Is the covariance matrix correct?");
    eigenvalues = Eigen::Vector3d::Zero();      // Setting the scale to zero will hide it on the screen
    eigenvectors = Eigen::Matrix3d::Identity();
  }

  // Be sure we have a right-handed orientation system
  makeRightHanded(eigenvectors, eigenvalues);

  // Define the rotation
  orientation.FromRotationMatrix(Ogre::Matrix3(eigenvectors(0,0), eigenvectors(0,1), eigenvectors(0,2),
                                               eigenvectors(1,0), eigenvectors(1,1), eigenvectors(1,2),
                                               eigenvectors(2,0), eigenvectors(2,1), eigenvectors(2,2)));

  // Define the scale. eigenvalues are the variances, so we take the sqrt to draw the standard deviation
  scale.x = 2*std::sqrt (eigenvalues[0]);
  scale.y = 2*std::sqrt (eigenvalues[1]);
  scale.z = 2*std::sqrt (eigenvalues[2]);
}

enum Plane {
  YZ_PLANE, // normal is x-axis
  XZ_PLANE, // normal is y-axis
  XY_PLANE  // normal is z-axis
};

void computeShapeScaleAndOrientation2D(const Eigen::Matrix2d& covariance, Ogre::Vector3& scale, Ogre::Quaternion& orientation, Plane plane = XY_PLANE)
{
  Eigen::Vector2d eigenvalues(Eigen::Vector2d::Identity());
  Eigen::Matrix2d eigenvectors(Eigen::Matrix2d::Zero());

  // NOTE: The SelfAdjointEigenSolver only references the lower triangular part of the covariance matrix
  // FIXME: Should we use Eigen's pseudoEigenvectors() ?
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(covariance);
  // Compute eigenvectors and eigenvalues
  if (eigensolver.info () == Eigen::Success)
  {
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors = eigensolver.eigenvectors();
  }
  else
  {
    ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values for position. Is the covariance matrix correct?");
    eigenvalues = Eigen::Vector2d::Zero();      // Setting the scale to zero will hide it on the screen
    eigenvectors = Eigen::Matrix2d::Identity();
  }

  // Be sure we have a right-handed orientation system
  makeRightHanded(eigenvectors, eigenvalues);

  // Define the rotation and scale of the plane
  // The Eigenvalues are the variances. The scales are two times the standard
  // deviation. The scale of the missing dimension is set to zero.
  if(plane == YZ_PLANE)
  {
    orientation.FromRotationMatrix(Ogre::Matrix3(1,        0,                 0,
                                                 0, eigenvectors(0,0), eigenvectors(0,1),
                                                 0, eigenvectors(1,0), eigenvectors(1,1)));

    scale.x = 0;
    scale.y = 2*std::sqrt (eigenvalues[0]);
    scale.z = 2*std::sqrt (eigenvalues[1]);

  }
  else if(plane == XZ_PLANE)
  {
    orientation.FromRotationMatrix(Ogre::Matrix3(eigenvectors(0,0), 0, eigenvectors(0,1),
                                                        0,          1,        0,
                                                 eigenvectors(1,0), 0, eigenvectors(1,1)));

    scale.x = 2*std::sqrt (eigenvalues[0]);
    scale.y = 0;
    scale.z = 2*std::sqrt (eigenvalues[1]);
  }
  else // plane == XY_PLANE
  {
    orientation.FromRotationMatrix(Ogre::Matrix3(eigenvectors(0,0), eigenvectors(0,1), 0,
                                                 eigenvectors(1,0), eigenvectors(1,1), 0,
                                                        0,                 0,          1));

    scale.x = 2*std::sqrt (eigenvalues[0]);
    scale.y = 2*std::sqrt (eigenvalues[1]);
    scale.z = 0;
  }
}

void radianScaleToMetricScale(Ogre::Real & radian_scale)
{
  radian_scale /= 2.0;
  if(radian_scale > deg2rad(85.0)) radian_scale = deg2rad(85.0);
  radian_scale = 2.0 * tan(radian_scale);
}

void CovarianceVisual::setCovariance( const geometry_msgs::PoseWithCovariance& pose )
{
  // check for NaN in covariance
  for (unsigned i = 0; i < 3; ++i)
  {
      if(isnan(pose.covariance[i]))
      {
          ROS_WARN_THROTTLE(1, "covariance contains NaN");
          return;
      }
  }

  // store position in Ogre structures
  Ogre::Quaternion ori(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
  // The position node should follow the orientation of the fixed frame, so we set it here to the inverse of the message's orientation
  fixed_orientation_node_->setOrientation(ori.Inverse());
  // Map covariance to a Eigen::Matrix 
  Eigen::Map<const Eigen::Matrix<double,6,6> > covariance(pose.covariance.data());

  updatePosition(covariance);
  updateOrientation(covariance, kRoll);
  updateOrientation(covariance, kPitch);
  updateOrientation(covariance, kYaw);

}

void CovarianceVisual::updatePosition( const Eigen::Matrix6d& covariance )
{
  // Compute shape and orientation for the position part of covariance
  Ogre::Vector3 shape_scale;
  Ogre::Quaternion shape_orientation;
  // NOTE: we call the 3D version here.
  computeShapeScaleAndOrientation3D(covariance.topLeftCorner<3,3>(), shape_scale, shape_orientation);
  // Rotate and scale the position scene node
  position_node_->setOrientation(shape_orientation);
  if(!shape_scale.isNaN())
      position_node_->setScale(shape_scale);
  else
      ROS_WARN_STREAM("position shape_scale contains NaN: " << shape_scale);
}

void CovarianceVisual::updateOrientation( const Eigen::Matrix6d& covariance, BryanAngle angle )
{
  // Get the correct sub-matrix based on the angle
  Eigen::Matrix2d covarianceAxis;
  if(angle == kRoll)
  {
    covarianceAxis = covariance.bottomRightCorner<2,2>();
  }
  else if(angle == kPitch)
  {
    covarianceAxis << covariance(3,3), covariance(3,5), covariance(5,3), covariance(5,5);
  }
  else // angle == kYaw
  {
    covarianceAxis = covariance.block<2,2>(3,3);
  }

  Ogre::Vector3 shape_scale;
  Ogre::Quaternion shape_orientation;
  // Compute shape and orientation for the orientation shape
  // NOTE: The cylinder mesh is oriented along its y axis, thus we want to flat it out into the XZ plane
  computeShapeScaleAndOrientation2D(covarianceAxis, shape_scale, shape_orientation, XZ_PLANE);
  // The computed scale is equivalent to twice the standard deviation _in radians_.
  // So we need to convert it to the linear scale of the shape using tan().
  // Also, we bound the maximum std to 85 deg.
  radianScaleToMetricScale(shape_scale.x);
  radianScaleToMetricScale(shape_scale.z);
  // Give a minimal height for the cylinder for better visualization
  shape_scale.y = 0.001;

  // Rotate and scale the scene node of the orientation part
  orientation_node_[angle]->setOrientation(shape_orientation);
  if(!shape_scale.isNaN())
      orientation_node_[angle]->setScale(shape_scale);
  else
      ROS_WARN_STREAM("orientation shape_scale contains NaN: " << shape_scale);
}

void CovarianceVisual::setScales( float pos_scale, float ori_scale)
{
  setPositionScale(pos_scale);
  setOrientationScale(ori_scale);
}

void CovarianceVisual::setPositionScale( float pos_scale ) 
{
  position_scale_node_->setScale( pos_scale, pos_scale, pos_scale );
}

void CovarianceVisual::setOrientationScale( float ori_scale )
{
  orientation_scale_node_->setScale( ori_scale, ori_scale, ori_scale );
  for(int i = 0; i < 3; i++)
  {
    // The scale along the y-axis is always 1.0
    orientation_offset_node_[i]->setScale( ori_scale, 1.0, ori_scale );    
  }
}

void CovarianceVisual::setPositionColor(const Ogre::ColourValue& c)
{
  position_shape_->setColor(c);
}

void CovarianceVisual::setOrientationColor(const Ogre::ColourValue& c)
{
  // FIXME: Is it better fix to rgb color? Or give the option to the user
  for(int i = 0; i < 3; i++)
  {
    orientation_shape_[i]->setColor(c);
  }
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

void CovarianceVisual::setUserData( const Ogre::Any& data )
{
  position_shape_->setUserData( data );
  for(int i = 0; i < 3; i++)
  {
    orientation_shape_[i]->setUserData( data );
  }
}

void CovarianceVisual::setPositionVisible( bool visible )
{
  position_node_->setVisible( visible );
}

void CovarianceVisual::setOrientationVisible( bool visible )
{
  orientation_scale_node_->setVisible( visible );
}

const Ogre::Vector3& CovarianceVisual::getPosition() 
{
  return position_node_->getPosition();
}

const Ogre::Quaternion& CovarianceVisual::getOrientation()
{
  return position_node_->getOrientation();
}

void CovarianceVisual::setPosition( const Ogre::Vector3& position )
{
  root_node_->setPosition( position );
}

void CovarianceVisual::setOrientation( const Ogre::Quaternion& orientation )
{
  root_node_->setOrientation( orientation );
}

void CovarianceVisual::setRotatingFrame( bool is_local_rotation )
{
  if(local_rotation_ == is_local_rotation)
    return;

  local_rotation_ = is_local_rotation;

  if(local_rotation_)
    root_node_->addChild( fixed_orientation_node_->removeChild( orientation_scale_node_->getName() ) );
  else
    fixed_orientation_node_->addChild( root_node_->removeChild( orientation_scale_node_->getName() ) );

}

rviz::Shape* CovarianceVisual::getOrientationShape(BryanAngle angle)
{
  return orientation_shape_[angle];
}

} // namespace rviz_plugin_covariance
