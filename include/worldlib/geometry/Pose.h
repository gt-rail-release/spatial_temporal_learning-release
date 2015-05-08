/*!
 * \file Pose.h
 * \brief Position and orientation information.
 *
 * A pose contains position and orientation information. This class is useful for internal data management within the
 * worldlib library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_GEOMETRY_POSE_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_GEOMETRY_POSE_H_

// worldlib
#include "Orientation.h"
#include "Position.h"

// ROS
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Transform.h>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace geometry
{

/*!
 * \class Pose
 * \brief Position and orientation information.
 *
 * A pose contains position and orientation information. This class is useful for internal data management within the
 * worldlib library. Convenience functions are added for use with ROS messages.
 */
class Pose
{
public:
  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the given position and orientation.
   *
   * \param position The position values of this Pose (defaults are 0).
   * \param orientation The orientation values of this Pose (defaults are a 0 rotation).
   */
  Pose(const Position &position = Position(), const Orientation &orientation = Orientation());

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the pose data from the given ROS message.
   *
   * \param pose The ROS Pose message to extract position and orientation data from.
   */
  Pose(const geometry_msgs::Pose &pose);

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the data from the given ROS Transform message.
   *
   * \param position The ROS Transform message to extract position and orientation data from.
   */
  Pose(const geometry_msgs::Transform &tf);

  /*!
   * \brief Create a new Pose.
   *
   * Creates a new Pose with the data from the given ROS tf2 Transform.
   *
   * \param position The ROS tf2 Transform to extract position and orientation data from.
   */
  Pose(const tf2::Transform &tf);

  /*!
   * \brief Position value mutator.
   *
   * Set the position value of this Pose.
   *
   * \param position The new position value.
   */
  void setPosition(const Position &position);

  /*!
   * \brief Position value accessor (immutable).
   *
   * Get the position value of this Pose.
   *
   * \return The position value.
   */
  const Position &getPosition() const;

  /*!
   * \brief Position value accessor.
   *
   * Get the position value of this Pose.
   *
   * \return The position value.
   */
  Position &getPosition();

  /*!
   * \brief Orientation value mutator.
   *
   * Set the orientation value of this Pose.
   *
   * \param orientation The new orientation value.
   */
  void setOrientation(const Orientation &orientation);

  /*!
   * \brief Orientation value accessor (immutable).
   *
   * Get the orientation value of this Pose.
   *
   * \return The orientation value.
   */
  const Orientation &getOrientation() const;

  /*!
   * \brief Orientation value accessor.
   *
   * Get the orientation value of this Pose.
   *
   * \return The orientation value.
   */
  Orientation &getOrientation();

  /*!
   * Converts this Pose object into a ROS Pose message.
   *
   * \return The ROS Pose message with this pose data.
   */
  geometry_msgs::Pose toROSPoseMessage() const;

  /*!
   * Converts this Pose object into a ROS Transform message.
   *
   * \return The ROS Transform message with this positional data.
   */
  geometry_msgs::Transform toROSTransformMessage() const;

  /*!
   * Converts this Pose object into a ROS tf2 Transform.
   *
   * \return The ROS tf2 Transform with this positional data.
   */
  tf2::Transform toTF2Transform() const;

private:
  /*! Position data. */
  Position position_;
  /*! Orientation data. */
  Orientation orientation_;
};

}
}
}
}

#endif
