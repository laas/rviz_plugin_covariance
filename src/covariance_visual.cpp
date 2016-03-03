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

double deg2rad (double degrees) {
    return degrees * 4.0 * atan (1.0) / 180.0;
}

CovarianceVisual::CovarianceVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, bool is_visible, float pos_scale, float ori_scale)
: Object( scene_manager ),
  position_scale_factor_( 1.0f ), orientation_scale_factor_( 1.0f ),
  position_msg_scale_(new Ogre::Vector3(0.0f,0.0f,0.0f)),
  orientation_x_msg_scale_(new Ogre::Vector3(0.0f,0.0f,0.0f)),
  orientation_y_msg_scale_(new Ogre::Vector3(0.0f,0.0f,0.0f)),
  orientation_z_msg_scale_(new Ogre::Vector3(0.0f,0.0f,0.0f))
{
  frame_node_ = parent_node->createChildSceneNode();

  position_node_ = frame_node_->createChildSceneNode();
  position_shape_ = new rviz::Shape(rviz::Shape::Sphere, scene_manager_, position_node_);

  orientation_x_node_ = frame_node_->createChildSceneNode();
  orientation_x_shape_ = new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, orientation_x_node_);

  orientation_y_node_ = frame_node_->createChildSceneNode();
  orientation_y_shape_ = new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, orientation_y_node_);

  orientation_z_node_ = frame_node_->createChildSceneNode();
  orientation_z_shape_ = new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, orientation_z_node_);

  setVisible( is_visible );

  setScales( pos_scale, ori_scale );
}

CovarianceVisual::~CovarianceVisual()
{
  delete position_shape_;
  delete orientation_x_shape_;
  delete orientation_y_shape_;
  delete orientation_z_shape_;

  scene_manager_->destroySceneNode( position_node_->getName() );
  scene_manager_->destroySceneNode( orientation_x_node_->getName() );
  scene_manager_->destroySceneNode( orientation_y_node_->getName() );
  scene_manager_->destroySceneNode( orientation_z_node_->getName() );
  scene_manager_->destroySceneNode( frame_node_->getName() );
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

// void set3DOrientationShape(const Eigen::Matrix2d& cov, msg_scale, node, position, orientation, offset, axis_alignment)
void setOrientationShape(
  const Eigen::Matrix2d& cov, Ogre::Vector3& msg_scale, Ogre::SceneNode* node, float scale_factor,
  const Ogre::Vector3& position, const Ogre::Quaternion& orientation,
  const Ogre::Vector3& offset, const Ogre::Quaternion& axis_alignment)
{
  Ogre::Vector3 shape_scale;
  Ogre::Quaternion shape_orientation;
  // Compute shape and orientation for the orientation shape
  // NOTE: The cylinder mesh is oriented along its y axis, thus we want to flat it out into the XZ plane
  computeShapeScaleAndOrientation2D(cov, shape_scale, shape_orientation, XZ_PLANE);
  // The computed scale is equivalent to twice the standard deviation _in radians_.
  // So we need to convert it to the linear scale of the shape using tan().
  // Also, we bound the maximum std to 85 deg.
  radianScaleToMetricScale(shape_scale.x);
  radianScaleToMetricScale(shape_scale.z);
  // Give a minimal height for the cylinder for better visualization
  shape_scale.y = 0.001;
  // store the shape (needed if the scale factor changes later)
  // Note it's returned by parameter
  msg_scale = shape_scale;
  // Apply the scale factor
  shape_scale *= scale_factor;

  // Position, rotate and scale the scene node of the orientation part
  // Note we position the cylinder along an axis using a offset
  node->setPosition(position + orientation * (scale_factor * offset));
  // Note the shape_orientation is composed with the orientation.
  // The axis_aligment should make the cylinder perpendicular to the axis
  node->setOrientation(orientation * axis_alignment * shape_orientation);

  // FIXME: The following lines can be used if the message rotation is being represented in the static frame
  // node->setPosition(position + scale_factor * offset);
  // node->setOrientation(axis_alignment * shape_orientation);

  if(!shape_scale.isNaN())
      node->setScale(shape_scale);
  else
      ROS_WARN_STREAM("orientation shape_scale contains NaN: " << shape_scale);

}

// This method compute the eigenvalues and eigenvectors of the position and orientation part covariance matrix
// separatelly and use their values to rotate and scale the covarance shapes.
// WARNING: rotations can be defined in the static frame (header.frame_id of a stamped message) or in the 
//          rotational (local, child_frame_id of a Odometry message). The implemented method below consider 
//          the rotations as being defined in the LOCAL frame. It won't work properly in the other case.
// FIXME: Consider orientations being expressed in both static and rotational frames
void CovarianceVisual::setCovariance( const geometry_msgs::PoseWithCovariance& message )
{
  // check for NaN in covariance
  for (unsigned i = 0; i < 3; ++i)
  {
      if(isnan(message.covariance[i]))
      {
          ROS_WARN_THROTTLE(1, "covariance contains NaN");
          return;
      }
  }

  // store pose in Ogre structures
  Ogre::Vector3 msg_position(message.pose.position.x,message.pose.position.y,message.pose.position.z);
  Ogre::Quaternion msg_orientation(message.pose.orientation.w, message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z);

  Eigen::Map<const Eigen::Matrix<double,6,6> > covariance(message.covariance.data());

  // Compute shape and orientation for the position part of covariance
  Ogre::Vector3 shape_scale;
  Ogre::Quaternion shape_orientation;
  // NOTE: we call the 3D version here.
  computeShapeScaleAndOrientation3D(covariance.topLeftCorner<3,3>(), shape_scale, shape_orientation);
  // store the shape (needed if the scale factor changes later)
  (*position_msg_scale_) = shape_scale;
  // update the shape by the scale factor
  shape_scale *= position_scale_factor_;

  // Position, rotate and scale the scene node of the position part
  position_node_->setPosition(msg_position);
  position_node_->setOrientation(shape_orientation);
  if(!shape_scale.isNaN())
      position_node_->setScale(shape_scale);
  else
      ROS_WARN_STREAM("position shape_scale contains NaN: " << shape_scale);

  // Set shape and orientation for the orientation shape on the x-axis (roll)
  setOrientationShape(
    covariance.bottomRightCorner<2,2>(),
    (*orientation_x_msg_scale_),
    orientation_x_node_,
    orientation_scale_factor_,
    msg_position,
    msg_orientation,
    Ogre::Vector3::UNIT_X,
    Ogre::Quaternion( Ogre::Degree( 90 ), Ogre::Vector3::UNIT_X ) * Ogre::Quaternion( Ogre::Degree( 90 ), Ogre::Vector3::UNIT_Z ));

  // Repeat the same computation for the other axes
  // y-axis (pitch)
  Eigen::Matrix2d cov_roll_yaw;
  cov_roll_yaw << covariance(3,3), covariance(3,5), covariance(5,3), covariance(5,5);
  setOrientationShape(
    cov_roll_yaw,
    (*orientation_y_msg_scale_),
    orientation_y_node_,
    orientation_scale_factor_,
    msg_position,
    msg_orientation,
    Ogre::Vector3::UNIT_Y,
    Ogre::Quaternion( Ogre::Degree( 90 ), Ogre::Vector3::UNIT_Y ));

  // z-axis (yaw)
  setOrientationShape(
    covariance.block<2,2>(3,3),
    (*orientation_z_msg_scale_),
    orientation_z_node_,
    orientation_scale_factor_,
    msg_position,
    msg_orientation,
    Ogre::Vector3::UNIT_Z,
    Ogre::Quaternion( Ogre::Degree( 90 ), Ogre::Vector3::UNIT_X ));

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
  orientation_x_node_->setScale((*orientation_x_msg_scale_) * orientation_scale_factor_);
  orientation_y_node_->setScale((*orientation_y_msg_scale_) * orientation_scale_factor_);
  orientation_z_node_->setScale((*orientation_z_msg_scale_) * orientation_scale_factor_);
  // TODO: Correct positions here as well
}

void CovarianceVisual::setPositionColor(const Ogre::ColourValue& c)
{
  position_shape_->setColor(c);
}

void CovarianceVisual::setOrientationColor(const Ogre::ColourValue& c)
{
  // FIXME: Is it better fix to rgb color? Or give the option to the user
  orientation_x_shape_->setColor(c);
  orientation_y_shape_->setColor(c);
  orientation_z_shape_->setColor(c);
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
  orientation_x_shape_->setUserData( data );
  orientation_y_shape_->setUserData( data );
  orientation_z_shape_->setUserData( data );
}

void CovarianceVisual::setVisible( bool visible )
{
  frame_node_->setVisible( visible );
}

const Ogre::Vector3& CovarianceVisual::getPosition() 
{
  return position_node_->getPosition();
}

const Ogre::Quaternion& CovarianceVisual::getOrientation()
{
  return position_node_->getOrientation();
}

void CovarianceVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
}

void CovarianceVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
}

} // namespace rviz

