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

// Anonymous namespace with helper functions
namespace
{
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> computeEigenValuesAndVectors
                                                (CovarianceVisual::covariance_type msg_covariance,
                                                 unsigned offset)
    {
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
        Eigen::Vector3d eigenValues = Eigen::Vector3d::Identity();
        Eigen::Matrix3d eigenVectors = Eigen::Matrix3d::Zero();

        for(unsigned i = 0; i < 3; ++i)
            for(unsigned j = 0; j < 3; ++j)
                covariance(i, j) = msg_covariance[(i + offset) * 6 + j + offset];

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
                     (std::pair<Eigen::Matrix3d, Eigen::Vector3d>& pair)
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


CovarianceVisual::CovarianceVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node, float pos_scale, float ori_scale)
: Object( scene_manager ),
  position_scale_( 1.0f ), orientation_scale_( 1.0f )
{

  position_node_ = parent_node->createChildSceneNode();
  position_shape_ = new rviz::Shape(rviz::Shape::Sphere, scene_manager_, position_node_);

  orientation_node_ = parent_node->createChildSceneNode();
  orientation_shape_ = new rviz::Shape(rviz::Shape::Cone, scene_manager_, orientation_node_);

  setScales( pos_scale, ori_scale );

}

CovarianceVisual::~CovarianceVisual()
{
  delete position_shape_;
  delete orientation_shape_;

  scene_manager_->destroySceneNode( position_node_->getName() );
  scene_manager_->destroySceneNode( orientation_node_->getName() );
}

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

    // Compute eigen values and vectors for both shapes.
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> positionEigenVectorsAndValues(computeEigenValuesAndVectors(msg_covariance, 0));
    std::pair<Eigen::Matrix3d, Eigen::Vector3d> orientationEigenVectorsAndValues(computeEigenValuesAndVectors(msg_covariance, 3));

    Ogre::Quaternion positionQuaternion(computeRotation(positionEigenVectorsAndValues));
    Ogre::Quaternion orientationQuaternion(computeRotation(orientationEigenVectorsAndValues));

    position_node_->setOrientation(positionQuaternion);
    orientation_node_->setOrientation(orientationQuaternion);

    // Compute scaling.
    //Ogre::Vector3 axesScaling(1, 1, 1);

    //axesScaling *= scaleFactor_;

    Ogre::Vector3 positionScaling
                  (std::sqrt (positionEigenVectorsAndValues.second[0]),
                   std::sqrt (positionEigenVectorsAndValues.second[1]),
                   std::sqrt (positionEigenVectorsAndValues.second[2]));

    positionScaling *= position_scale_;

    Ogre::Vector3 orientationScaling
                  (std::sqrt (orientationEigenVectorsAndValues.second[0]),
                   std::sqrt (orientationEigenVectorsAndValues.second[1]),
                   std::sqrt (orientationEigenVectorsAndValues.second[2]));

    orientationScaling *= orientation_scale_;

    // Set the scaling.
    /*if(!axesScaling.isNaN())
        axes_->setScale(axesScaling);
    else
        ROS_WARN_STREAM("axesScaling contains NaN: " << axesScaling);*/

    if(!positionScaling.isNaN())
        position_node_->setScale(positionScaling);
    else
        ROS_WARN_STREAM("positionScaling contains NaN: " << positionScaling);

    if(!orientationScaling.isNaN())
        orientation_node_->setScale(orientationScaling);
    else
        ROS_WARN_STREAM("orientationScaling contains NaN: " << orientationScaling);

    // Debugging.
    ROS_DEBUG_STREAM_THROTTLE
    (1.,
    "Positional part 3x3 eigen values:\n"
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

