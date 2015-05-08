/*!
 * \file Observation.h
 * \brief World observation configuration information.
 *
 * An observation consists of seeing a given item on a given surface at a give pose.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 24, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_OBSERVATION_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_OBSERVATION_H_

// worldlib
#include "Item.h"
#include "PlacementSurface.h"
#include "Surface.h"
#include "../geometry/Pose.h"

// ROS
#include <ros/time.h>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace world
{

/*!
 * \class Observation
 * \brief World observation configuration information.
 *
 * An observation consists of seeing a given item on a given surface at a give pose.
 */
class Observation
{
public:
  /*!
   * \brief Create a new Observation.
   *
   * Create a new Observation with the given Item, Surface, and Pose (in reference to the surface itself).
   *
   * \param item The Item seen in this Observation (defaults to empty Item).
   * \param surface The Surface the Item was seen on in this Observation (defaults to empty Surface).
   * \param pose The Pose of this Item in this Observation with respect to the Surface (defaults to empty Pose).
   * \param time The ros::Time the observation was made (defaults to now).
   * \param removed_estimate The estimated ros::Time the Item was removed from the Surface (defaults to 0).
   * \param removed_observed The observed ros::Time the Item was not seen on the Surface (defaults to 0).
   */
  Observation(const Item &item = Item(), const Surface &surface = Surface(),
      const geometry::Pose &pose = geometry::Pose(), const ros::Time &time = ros::Time::now(),
      const ros::Time &removed_estimate = ros::Time(0), const ros::Time &removed_observed = ros::Time(0));

  /*!
   * \brief Item value accessor (immutable).
   *
   * Get the Item value of this Observation.
   *
   * \return The Item value of this Observation.
   */
  const Item &getItem() const;

  /*!
   * \brief Item value accessor.
   *
   * Get the Item value of this Observation.
   *
   * \return The Item value of this Observation.
   */
  Item &getItem();

  /*!
   * \brief Item value mutator.
   *
   * Set the Item value of this Observation.
   *
   * \param item The new Item value of this Observation.
   */
  void setItem(const Item &item);

  /*!
   * \brief Surface value accessor (immutable).
   *
   * Get the Surface value of this Observation.
   *
   * \return The Surface value of this Observation.
   */
  const Surface &getSurface() const;

  /*!
   * \brief Surface value accessor.
   *
   * Get the Surface value of this Observation.
   *
   * \return The Surface value of this Observation.
   */
  Surface &getSurface();

  /*!
   * \brief Surface value mutator.
   *
   * Set the Surface value of this Observation.
   *
   * \param surface The new Surface value of this Observation.
   */
  void setSurface(const Surface &surface);

  /*!
   * \brief PlacementSurface accessor.
   *
   * Get the PlacementSurface of this Observation (defined as the closest placement surface to the observation Pose).
   *
   * \return The PlacementSurface value of this Observation.
   */
  const PlacementSurface &getPlacementSurface() const;

  /*!
   * \brief Pose value accessor (immutable).
   *
   * Get the Pose value of this Observation.
   *
   * \return The Pose value of this Observation.
   */
  const geometry::Pose &getPose() const;

  /*!
   * \brief Pose value accessor.
   *
   * Get the Pose value of this Observation.
   *
   * \return The Pose value of this Observation.
   */
  geometry::Pose &getPose();

  /*!
   * \brief Pose value mutator.
   *
   * Set the Pose value of this Observation.
   *
   * \param pose The new Pose value of this Observation.
   */
  void setPose(const geometry::Pose &pose);

  /*!
   * \brief Time value accessor (immutable).
   *
   * Get the time value of this Observation.
   *
   * \return The time value of this Observation.
   */
  const ros::Time &getTime() const;

  /*!
   * \brief Time value accessor.
   *
   * Get the time value of this Observation.
   *
   * \return The time value of this Observation.
   */
  ros::Time &getTime();

  /*!
   * \brief Time value mutator.
   *
   * Set the time value of this Observation.
   *
   * \param time The new time value of this Observation.
   */
  void setTime(const ros::Time &time);

  /*!
   * \brief Estimated removal time value accessor (immutable).
   *
   * Get the estimated removal time value of this Observation.
   *
   * \return The estimated removal time value of this Observation.
   */
  const ros::Time &getRemovedEstimate() const;

  /*!
   * \brief Estimated removal time value accessor.
   *
   * Get the estimated removal time value of this Observation.
   *
   * \return The estimated removal time value of this Observation.
   */
  ros::Time &getRemovedEstimate();

  /*!
   * \brief Estimated removal time value mutator.
   *
   * Set the estimated removal time value of this Observation.
   *
   * \param time The new estimated removal time value of this Observation.
   */
  void setRemovedEstimate(const ros::Time &removed_estimate);

  /*!
   * \brief Observed removal time value accessor (immutable).
   *
   * Get the observed removal time value of this Observation.
   *
   * \return The observed removal time value of this Observation.
   */
  const ros::Time &getRemovedObserved() const;

  /*!
   * \brief Observed removal time value accessor.
   *
   * Get the observed removal time value of this Observation.
   *
   * \return The observed removal time value of this Observation.
   */
  ros::Time &getRemovedObserved();

  /*!
   * \brief Observed removal time value mutator.
   *
   * Set the observed removal time value of this Observation.
   *
   * \param time The new observed removal time value of this Observation.
   */
  void setRemovedObserved(const ros::Time &removed_observed);

private:
  /*! The Item in this Observation. */
  Item item_;
  /*! The Surface in this Observation. */
  Surface surface_;
  /*! The Pose in this Observation. */
  geometry::Pose pose_;
  /*! The various timestamps in this Observation. */
  ros::Time time_, removed_estimate_, removed_observed_;
};

}
}
}
}

#endif
