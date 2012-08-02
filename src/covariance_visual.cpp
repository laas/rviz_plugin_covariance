#include <Eigen/Dense>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>

#include "covariance_visual.h"

namespace rviz_plugin_covariance
{
  CovarianceVisual::CovarianceVisual (Ogre::SceneManager* scene_manager,
				      Ogre::SceneNode* parent_node)
  {
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
    shape_ = new rviz::Shape
      (rviz::Shape::Sphere, scene_manager_, frame_node_);
  }

  CovarianceVisual::~CovarianceVisual ()
  {
    delete shape_;
    scene_manager_->destroySceneNode (frame_node_);
  }

  void
  CovarianceVisual::setMessage
  (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    const geometry_msgs::Point& p = msg->pose.pose.position;

    // Copy position.
    Ogre::Vector3 position (p.x, p.y, p.z);
    shape_->setPosition (position);

    Eigen::Matrix4d covariance;
    Eigen::Vector4d eigenValues;
    Eigen::Matrix4d eigenVectors;

    // Copy covariance.
    for (unsigned i = 0; i < 4; ++i)
      for (unsigned j = 0; j < 4; ++j)
	covariance (i, j) = msg->pose.covariance[i * 4 + j];
    
    // Compute eigen values and eigen vectors.
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver (covariance);

    if (eigensolver.info () == Eigen::Success) 
      {
	eigenValues = eigensolver.eigenvalues();
	eigenVectors = eigensolver.eigenvectors();
      }

    // Compute orientation based on the eigen vectors.
    Ogre::Quaternion q; //FIXME: fix this!
    shape_->setOrientation (q);

    // Scale depending on the eigen value.
    Ogre::Vector3 scale
      (eigenValues[0] * 2, eigenValues[1] * 2, eigenValues[2] * 2);
    shape_->setScale (scale);
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
  }
} // end namespace rviz_plugin_covariance
