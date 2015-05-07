/*!
 * \file Orientation.h
 * \brief Quaternion orientation information.
 *
 * An orientation simply contains x, y, z, and w values. This class is useful for internal data management within
 * the worldlib library. Convenience functions are added for use with ROS messages.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_GEOMETRY_ORIENTATION_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_GEOMETRY_ORIENTATION_H_

// ROS
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace geometry
{

/*!
 * \class Orientation
 * \brief Quaternion orientation information.
 *
 * An orientation simply contains x, y, z, and w values. This class is useful for internal data management within
 * the worldlib library. Convenience functions are added for use with ROS messages.
 */
class Orientation
{
public:
  /*!
   * \brief Create a new Orientation.
   *
   * Creates a new Orientation with the theta value about the Z axis.
   *
   * \param theta The theta value.
   */
  Orientation(const double theta = 0);

  /*!
   * \brief Create a new Orientation.
   *
   * Creates a new Orientation with the given x, y, z, and w values.
   *
   * \param x The x value.
   * \param y The y value.
   * \param z The z value.
   * \param w The w value.
   */
  Orientation(const double x, const double y, const double z, const double w);

  /*!
   * \brief Create a new Orientation.
   *
   * Creates a new Orientation with the given x, y, z, and w values from the ROS Quaternion message.
   *
   * \param point The ROS Quaternion message to extract values from.
   */
  Orientation(const geometry_msgs::Quaternion &quaternion);

  /*!
   * \brief Create a new Orientation.
   *
   * Creates a new Orientation with the given x, y, z, and w values from the ROS tf2 Quaternion.
   *
   * \param point The ROS tf2 Quaternion to extract values from.
   */
  Orientation(const tf2::Quaternion &quaternion);

  /*!
   * \brief X value mutator.
   *
   * Set the x value of this Orientation.
   *
   * \param x The new x value.
   */
  void setX(const double x);

  /*!
   * \brief X value accessor.
   *
   * Get the x value of this Orientation.
   *
   * \return The x value.
   */
  double getX() const;

  /*!
   * \brief Y value mutator.
   *
   * Set the y value of this Orientation.
   *
   * \param y The new y value.
   */
  void setY(const double y);

  /*!
   * \brief Y value accessor.
   *
   * Get the y value of this Orientation.
   *
   * \return The y value.
   */
  double getY() const;

  /*!
   * \brief Z value mutator.
   *
   * Set the z value of this Orientation.
   *
   * \param z The new z value.
   */
  void setZ(const double z);

  /*!
   * \brief Z value accessor.
   *
   * Get the z value of this Orientation.
   *
   * \return The z value.
   */
  double getZ() const;

  /*!
   * \brief W value mutator.
   *
   * Set the w value of this Orientation.
   *
   * \param w The new w value.
   */
  void setW(const double w);

  /*!
   * \brief W value accessor.
   *
   * Get the w value of this Orientation.
   *
   * \return The w value.
   */
  double getW() const;

  /*!
   * \brief Theta value about the z-axis accessor.
   *
   * Get the theta value about the z-axis of this Orientation.
   *
   * \return The theta value about the z-axis.
   */
  double getTheta() const;

  /*!
   * Converts this Orientation object into a ROS Quaternion message.
   *
   * \return The ROS Quaternion message with this orientation data.
   */
  geometry_msgs::Quaternion toROSQuaternionMessage() const;

  /*!
   * Converts this Orientation object into a ROS tf2 Quaternion.
   *
   * \return The ROS tf2 Quaternion with this orientation data.
   */
  tf2::Quaternion toTF2Quaternion() const;

  /*!
   * Converts this Orientation object into a ROS tf2 Matrix3x3.
   *
   * \return The ROS tf2 Matrix3x3 with this orientation data.
   */
  tf2::Matrix3x3 toTF2Matrix3x3() const;

private:
  /*! Members to hold values. */
  double x_, y_, z_, w_;
};

}
}
}
}

#endif
