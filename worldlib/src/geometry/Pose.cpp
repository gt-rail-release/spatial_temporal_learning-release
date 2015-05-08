/*!
 * \file Pose.cpp
 * \brief Position and orientation information.
 *
 * A pose contains position and orientation information. This class is useful for internal data management within the
 * worldlib library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

// worldlib
#include "worldlib/geometry/Pose.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::geometry;

Pose::Pose(const Position &position, const Orientation &orientation) : position_(position), orientation_(orientation)
{
}

Pose::Pose(const geometry_msgs::Pose &pose) : position_(pose.position), orientation_(pose.orientation)
{
}

Pose::Pose(const geometry_msgs::Transform &tf) : position_(tf.translation), orientation_(tf.rotation)
{
}

Pose::Pose(const tf2::Transform &tf) : position_(tf.getOrigin()), orientation_(tf.getRotation())
{
}

void Pose::setPosition(const Position &position)
{
  position_ = position;
}

const Position &Pose::getPosition() const
{
  return position_;
}

Position &Pose::getPosition()
{
  return position_;
}

void Pose::setOrientation(const Orientation &orientation)
{
  orientation_ = orientation;
}

const Orientation &Pose::getOrientation() const
{
  return orientation_;
}

Orientation &Pose::getOrientation()
{
  return orientation_;
}

geometry_msgs::Pose Pose::toROSPoseMessage() const
{
  geometry_msgs::Pose pose;
  pose.position = position_.toROSPointMessage();
  pose.orientation = orientation_.toROSQuaternionMessage();
  return pose;
}

tf2::Transform Pose::toTF2Transform() const
{
  tf2::Quaternion orientation = orientation_.toTF2Quaternion();
  tf2::Vector3 translation = position_.toTF2Vector3();
  tf2::Transform tf(orientation, translation);
  return tf;
}
