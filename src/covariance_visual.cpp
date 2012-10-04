#include <Eigen/Dense>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <ros/console.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/shape.h>

#include "covariance_visual.h"

namespace rviz_plugin_covariance
{
  CovarianceVisual::CovarianceVisual (Ogre::SceneManager* scene_manager,
				      Ogre::SceneNode* parent_node)
    : axes_ (),
      shape_ (),
      orientationShape_ (),
      frame_node_ (parent_node->createChildSceneNode()),
      positionNode_ (),
      orientationNode_ (),
      scene_manager_ (scene_manager),
      scaleFactor_ (1.)
  {
    axes_ = new rviz::Axes
      (scene_manager_, frame_node_);

    positionNode_ = axes_->getSceneNode ()->createChildSceneNode ();
    orientationNode_ = axes_->getSceneNode ()->createChildSceneNode();

    shape_ = new rviz::Shape
      (rviz::Shape::Sphere, scene_manager_, positionNode_);
    orientationShape_ = new rviz::Shape
      (rviz::Shape::Cone, scene_manager_, orientationNode_);
  }

  CovarianceVisual::~CovarianceVisual ()
  {
    delete shape_;
    delete orientationShape_;
    scene_manager_->destroySceneNode (orientationNode_);
    scene_manager_->destroySceneNode (positionNode_);
    delete axes_;
    scene_manager_->destroySceneNode (frame_node_);
  }

  namespace
  {
    std::pair<Eigen::Matrix3d, Eigen::Vector3d>
    computeEigenValuesAndVectors
    (const geometry_msgs::PoseWithCovariance& msg,
     unsigned offset)
    {
      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero ();
      Eigen::Vector3d eigenValues = Eigen::Vector3d::Identity ();
      Eigen::Matrix3d eigenVectors = Eigen::Matrix3d::Zero ();

      for (unsigned i = 0; i < 3; ++i)
	for (unsigned j = 0; j < 3; ++j)
	  covariance (i, j) =
	    msg.covariance[(i + offset) * 6 + j + offset];

      // Compute eigen values and eigen vectors.
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver
	(covariance);

      if (eigensolver.info () == Eigen::Success)
	{
	  eigenValues = eigensolver.eigenvalues();
	  eigenVectors = eigensolver.eigenvectors();
	}
      else
	ROS_WARN_THROTTLE
	  (1,
	   "failed to compute eigen vectors/values."
	   "Is the covariance matrix correct?");

      return std::make_pair (eigenVectors, eigenValues);

    }

    Ogre::Quaternion computeRotation
    (const geometry_msgs::PoseWithCovariance& msg,
     std::pair<Eigen::Matrix3d, Eigen::Vector3d>& pair)
    {
      Ogre::Matrix3 rotation;
      for (unsigned i = 0; i < 3; ++i)
	for (unsigned j = 0; j < 3; ++j)
	  rotation[i][j] = pair.first (i, j);
      return Ogre::Quaternion (rotation);
    }
  } // end of anonymous namespace.

  void
  CovarianceVisual::setMessage
  (const geometry_msgs::PoseWithCovariance& msg)
  {
    // Construct pose position and orientation.
    const geometry_msgs::Point& p = msg.pose.position;
    Ogre::Vector3 position (p.x, p.y, p.z);
    Ogre::Quaternion orientation
      (msg.pose.orientation.w,
       msg.pose.orientation.x,
       msg.pose.orientation.y,
       msg.pose.orientation.z);

    // Set position and orientation for axes scene node.
    axes_->setPosition (position);
    axes_->setOrientation (orientation);


    // Compute eigen values and vectors for both shapes.
    std::pair<Eigen::Matrix3d, Eigen::Vector3d>
      positionEigenVectorsAndValues
      (computeEigenValuesAndVectors (msg, 0));
    std::pair<Eigen::Matrix3d, Eigen::Vector3d>
      orientationEigenVectorsAndValues
      (computeEigenValuesAndVectors (msg, 3));

    Ogre::Quaternion positionQuaternion
      (computeRotation (msg, positionEigenVectorsAndValues));
    Ogre::Quaternion orientationQuaternion
      (computeRotation (msg, orientationEigenVectorsAndValues));

    positionNode_->setOrientation (positionQuaternion);
    orientationNode_->setOrientation (orientationQuaternion);

    // Compute scaling.
    Ogre::Vector3 positionScaling
      (std::sqrt (positionEigenVectorsAndValues.second[0]),
       std::sqrt (positionEigenVectorsAndValues.second[1]),
       std::sqrt (positionEigenVectorsAndValues.second[2]));
    positionScaling *= scaleFactor_;

    Ogre::Vector3 orientationScaling
      (std::sqrt (orientationEigenVectorsAndValues.second[0]),
       std::sqrt (orientationEigenVectorsAndValues.second[1]),
       std::sqrt (orientationEigenVectorsAndValues.second[2]));
    orientationScaling *= scaleFactor_;

    // Set the scaling.
    positionNode_->setScale (positionScaling);
    orientationNode_->setScale (orientationScaling);

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

  void
  CovarianceVisual::setMessage
  (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    setMessage (msg->pose);
  }

  void
  CovarianceVisual::setMessage
  (const nav_msgs::Odometry::ConstPtr& msg)
  {
    setMessage (msg->pose);
  }

  void
  CovarianceVisual::setFramePosition (const Ogre::Vector3& position)
  {
    frame_node_->setPosition (position);
  }

  void
  CovarianceVisual::setFrameOrientation (const Ogre::Quaternion& orientation)
  {
    frame_node_->setOrientation (orientation);
  }

  void CovarianceVisual::setColor (float r, float g, float b, float a)
  {
    shape_->setColor (r, g, b, a);
    orientationShape_->setColor (r, g, b, a);
  }
} // end namespace rviz_plugin_covariance
