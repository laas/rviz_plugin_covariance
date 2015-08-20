#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/shape.h>

#include "covariance_visual.h"

#include <ros/console.h>
#include <Eigen/Dense>

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


            if(Ogre::Quaternion(rotation).Norm() != 1.0f)
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

    CovarianceVisual::CovarianceVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node)
    {
        scene_manager_ = scene_manager;

        frame_node_ = parent_node->createChildSceneNode();

        axes_.reset(new rviz::Axes(scene_manager_, frame_node_));
        positionNode_ = axes_->getSceneNode()->createChildSceneNode ();
        orientationNode_ = axes_->getSceneNode()->createChildSceneNode();

        shape_.reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, positionNode_));
        orientationShape_.reset(new rviz::Shape(rviz::Shape::Cube, scene_manager_, orientationNode_));

        scaleFactor_covariance_ = 1.0f;
    }

    CovarianceVisual::~CovarianceVisual ()
    {
        scene_manager_->destroySceneNode (orientationNode_);
        scene_manager_->destroySceneNode (positionNode_);
        scene_manager_->destroySceneNode (frame_node_);
    }

    void CovarianceVisual::setMessage(const geometry_msgs::PoseWithCovariance& msg)
    {
        // Construct pose position and orientation.
        const geometry_msgs::Point& p = msg.pose.position;
        const geometry_msgs::Quaternion& q = msg.pose.orientation;

        Ogre::Vector3 position(p.x, p.y, p.z);
        Ogre::Quaternion orientation(q.w, q.x, q.y, q.z);

        // Set position and orientation for axes scene node.
        if(!position.isNaN())
            axes_->setPosition(position);
        else
            ROS_WARN_STREAM_THROTTLE(1, "position contains NaN: " << position);

        if(!orientation.isNaN())
            axes_->setOrientation (orientation);
        else
            ROS_WARN_STREAM_THROTTLE(1, "orientation contains NaN: " << orientation);

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

        positionNode_->setOrientation(positionQuaternion);
        orientationNode_->setOrientation(orientationQuaternion);

        // Compute scaling.
        Ogre::Vector3 axesScaling(1, 1, 1);

        axesScaling *= scaleFactor_axis_;

        Ogre::Vector3 positionScaling
                      (std::sqrt (positionEigenVectorsAndValues.second[0]),
                       std::sqrt (positionEigenVectorsAndValues.second[1]),
                       std::sqrt (positionEigenVectorsAndValues.second[2]));

        positionScaling *= scaleFactor_covariance_;

        Ogre::Vector3 orientationScaling
                      (std::sqrt (orientationEigenVectorsAndValues.second[0]),
                       std::sqrt (orientationEigenVectorsAndValues.second[1]),
                       std::sqrt (orientationEigenVectorsAndValues.second[2]));

        orientationScaling *= scaleFactor_covariance_;

        // Set the scaling.
        if(!axesScaling.isNaN())
            axes_->setScale(axesScaling);
        else
            ROS_WARN_STREAM("axesScaling contains NaN: " << axesScaling);

        if(!positionScaling.isNaN())
            positionNode_->setScale(positionScaling);
        else
            ROS_WARN_STREAM("positionScaling contains NaN: " << positionScaling);

        if(!orientationScaling.isNaN())
            orientationNode_->setScale(orientationScaling);
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

    void CovarianceVisual::setMessage(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        setMessage(msg->pose);
    }

    void
    CovarianceVisual::setFramePosition (const Ogre::Vector3& position)
    {
        frame_node_->setPosition(position);
    }

    void
    CovarianceVisual::setFrameOrientation (const Ogre::Quaternion& orientation)
    {
        frame_node_->setOrientation(orientation);
    }

    void CovarianceVisual::setColorPosition(float r, float g, float b, float a)
    {
        shape_->setColor(r, g, b, a);
    }

    void CovarianceVisual::setColorOrientation(float r, float g, float b, float a)
    {
        orientationShape_->setColor(r, g, b, a);
    }
} // end namespace rviz_plugin_covariance
