/*!
 * \file SpatialWorldObservation.h
 * \brief Observation information from an SQL entity in the spatial world database.
 *
 * An observation consists of seeing a given item on a given surface at a give pose. This class corresponds directly
 * to an entity in the spatial world database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 24, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_SPATIAL_WORLD_OBSERVATION_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_SPATIAL_WORLD_OBSERVATION_H_

// worldlib
#include "SqlEntity.h"
#include "../geometry/Pose.h"

// ROS
#include <ros/time.h>

// C++ Standard Library
#include <string>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace remote
{

/*!
 * \class SpatialWorldObservation
 * \brief Observation information from an SQL entity in the spatial world database.
 *
 * An observation consists of seeing a given item on a given surface at a give pose. This class corresponds directly
 * to an entity in the spatial world database.
 */
class SpatialWorldObservation : public SqlEntity
{
public:
  /*!
   * \brief Create a new SpatialWorldObservation.
   *
   * Create a new SpatialWorldObservation with the entity values.
   *
   * \param id The ID of the entity in the database.
   * \param item_name The name of the item observed.
   * \param surface_name The name of the surface the item was observed on.
   * \param surface_frame_id The frame ID of the surface the item was observed on.
   * \param pose The Pose of the item in this observation with respect to the surface frame ID.
   * \param time The ros::Time the observation was made
   * \param removed_estimate The estimated ros::Time the Item was removed from the surface.
   * \param removed_observed The observed ros::Time the Item was not seen on the surface.
   */
  SpatialWorldObservation(const uint32_t id, const std::string &item_name, const std::string &surface_name,
      const std::string &surface_frame_id, const geometry::Pose &pose, const ros::Time &time,
      const ros::Time &removed_estimate, const ros::Time &removed_observed);

  /*!
   * \brief Item name value accessor.
   *
   * Get the item name value of this SpatialWorldObservation.
   *
   * \return The item name value of this SpatialWorldObservation.
   */
  const std::string &getItemName() const;

  /*!
   * \brief Item value mutator.
   *
   * Set the item name value of this SpatialWorldObservation.
   *
   * \param item_name The new item name value of this SpatialWorldObservation.
   */
  void setItemName(const std::string &item_name);

  /*!
   * \brief Surface name value accessor.
   *
   * Get the surface name value of this SpatialWorldObservation.
   *
   * \return The surface name value of this SpatialWorldObservation.
   */
  const std::string &getSurfaceName() const;

  /*!
   * \brief Surface name value mutator.
   *
   * Set the surface name value of this SpatialWorldObservation.
   *
   * \param surface_name The new surface name value of this SpatialWorldObservation.
   */
  void setSurfaceName(const std::string &surface_name);

  /*!
   * \brief Surface frame ID value accessor.
   *
   * Get the surface frame ID value of this SpatialWorldObservation.
   *
   * \return The surface frame ID value of this SpatialWorldObservation.
   */
  const std::string &getSurfaceFrameID() const;

  /*!
   * \brief Surface frame ID value mutator.
   *
   * Set the surface frame ID value of this SpatialWorldObservation.
   *
   * \param surface_frame_id The new surface frame ID value of this SpatialWorldObservation.
   */
  void setSurfaceFrameID(const std::string &surface_frame_id);

  /*!
   * \brief Pose value accessor (immutable).
   *
   * Get the Pose value of this SpatialWorldObservation.
   *
   * \return The Pose value of this SpatialWorldObservation.
   */
  const geometry::Pose &getPose() const;

  /*!
   * \brief Pose value accessor.
   *
   * Get the Pose value of this SpatialWorldObservation.
   *
   * \return The Pose value of this SpatialWorldObservation.
   */
  geometry::Pose &getPose();

  /*!
   * \brief Pose value mutator.
   *
   * Set the Pose value of this SpatialWorldObservation.
   *
   * \param pose The new Pose value of this SpatialWorldObservation.
   */
  void setPose(const geometry::Pose &pose);

  /*!
   * \brief Time value accessor (immutable).
   *
   * Get the time value of this SpatialWorldObservation.
   *
   * \return The time value of this SpatialWorldObservation.
   */
  const ros::Time &getTime() const;

  /*!
   * \brief Time value accessor.
   *
   * Get the time value of this SpatialWorldObservation.
   *
   * \return The time value of this SpatialWorldObservation.
   */
  ros::Time &getTime();

  /*!
   * \brief Time value mutator.
   *
   * Set the time value of this SpatialWorldObservation.
   *
   * \param time The new time value of this SpatialWorldObservation.
   */
  void setTime(const ros::Time &time);

  /*!
   * \brief Estimated removal time value accessor (immutable).
   *
   * Get the estimated removal time value of this SpatialWorldObservation.
   *
   * \return The estimated removal time value of this SpatialWorldObservation.
   */
  const ros::Time &getRemovedEstimate() const;

  /*!
   * \brief Estimated removal time value accessor.
   *
   * Get the estimated removal time value of this SpatialWorldObservation.
   *
   * \return The estimated removal time value of this SpatialWorldObservation.
   */
  ros::Time &getRemovedEstimate();

  /*!
   * \brief Estimated removal time value mutator.
   *
   * Set the estimated removal time value of this SpatialWorldObservation.
   *
   * \param time The new estimated removal time value of this SpatialWorldObservation.
   */
  void setRemovedEstimate(const ros::Time &removed_estimate);

  /*!
   * \brief Observed removal time value accessor (immutable).
   *
   * Get the observed removal time value of this SpatialWorldObservation.
   *
   * \return The observed removal time value of this SpatialWorldObservation.
   */
  const ros::Time &getRemovedObserved() const;

  /*!
   * \brief Observed removal time value accessor.
   *
   * Get the observed removal time value of this SpatialWorldObservation.
   *
   * \return The observed removal time value of this SpatialWorldObservation.
   */
  ros::Time &getRemovedObserved();

  /*!
   * \brief Observed removal time value mutator.
   *
   * Set the observed removal time value of this SpatialWorldObservation.
   *
   * \param time The new observed removal time value of this SpatialWorldObservation.
   */
  void setRemovedObserved(const ros::Time &removed_observed);

private:
  /*! The various identification fields in the observation. */
  std::string item_name_, surface_name_, surface_frame_id_;
  /*! The Pose in this observation. */
  geometry::Pose pose_;
  /*! The various timestamps in this observation. */
  ros::Time time_, removed_estimate_, removed_observed_;
};

}
}
}
}

#endif
