/*!
 * \file GeoLifeEntry.h
 * \brief The GeoLife entry contains a timestamp with latitude and longitude information.
 *
 * An  GeoLife entry simply contains latitude and longitude information with an associated ROS timestamp.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 30, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_GEO_LIFE_ENTRY_H_
#define SPATIAL_TEMPORAL_LEARNING_GEO_LIFE_ENTRY_H_

// ROS
#include <ros/time.h>

namespace rail
{
namespace spatial_temporal_learning
{

/*!
 * \class GeoLifeEntry
 * \brief The GeoLife entry contains a timestamp with latitude and longitude information.
 *
 * An  GeoLife entry simply contains latitude and longitude information with an associated ROS timestamp.
 */
class GeoLifeEntry
{
public:
  /*!
   * \brief Create a new GeoLifeEntry.
   *
   * Create a new GeoLifeEntry with the given latitude, longitude, and associated ROS timestamp.
   *
   * \param latitude The latitude value (defaults to 0).
   * \param longitude The longitude value (defaults to 0).
   * \param time The ros::Time of the GeoLife data entry (defaults to 0 time).
   */
  GeoLifeEntry(const double latitude = 0, const double longitude = 0, const ros::Time &time = ros::Time(0));

  /*!
   * \brief Latitude value accessor.
   *
   * Get the latitude value of this GeoLifeEntry.
   *
   * \return The latitude value of this GeoLifeEntry.
   */
  double getLatitude() const;

  /*!
   * \brief Latitude value mutator.
   *
   * Set the latitude value of this GeoLifeEntry.
   *
   * \param latitude The new latitude value of this GeoLifeEntry.
   */
  void setLatitude(const double latitude);

  /*!
   * \brief Longitude value accessor.
   *
   * Get the longitude value of this GeoLifeEntry.
   *
   * \return The longitude value of this GeoLifeEntry.
   */
  double getLongitude() const;

  /*!
   * \brief Longitude value mutator.
   *
   * Set the longitude value of this GeoLifeEntry.
   *
   * \param longitude The new longitude value of this GeoLifeEntry.
   */
  void setLongitude(const double longitude);

  /*!
   * \brief Time value accessor (immutable).
   *
   * Get the time value of this GeoLifeEntry.
   *
   * \return The time value of this GeoLifeEntry.
   */
  const ros::Time &getTime() const;

  /*!
   * \brief Time value accessor.
   *
   * Get the time value of this GeoLifeEntry.
   *
   * \return The time value of this GeoLifeEntry.
   */
  ros::Time &getTime();

  /*!
   * \brief Time value mutator.
   *
   * Set the time value of this GeoLifeEntry.
   *
   * \param time The new time value of this GeoLifeEntry.
   */
  void setTime(const ros::Time &time);

private:
  /*! Latitude and longitude information. */
  double latitude_, longitude_;
  /*! The GeoLife timestamp value. */
  ros::Time time_;
};

}
}

#endif
