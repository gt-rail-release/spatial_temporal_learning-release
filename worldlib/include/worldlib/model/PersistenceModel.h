/*!
 * \file PersistenceModel.h
 * \brief Persistence model information.
 *
 * A persistence model contains information about the estimated time an item will disappear from the surface.
 * Persistence models are in the granularity of hours.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 28, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_MODEL_PERSISTENCE_MODEL_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_MODEL_PERSISTENCE_MODEL_H_

// worldlib
#include "../world/Item.h"
#include "../world/Surface.h"

// ROS
#include <ros/duration.h>
#include <ros/time.h>

// Boost
#include <boost/math/distributions/exponential.hpp>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace model
{

/*!
 * \class PersistenceModel
 * \brief Persistence model information.
 *
 * A persistence model contains information about the estimated time an item will disappear from the surface.
 * Persistence models are in the granularity of hours.
 */
class PersistenceModel
{
public:
  /*!
   * \brief Create a new PersistenceModel.
   *
   * Create a new PersistenceModel with the given parameters.
   *
   * \param item The Item of this PersistenceModel (defaults to empty Item).
   * \param surface The Surface of this PersistenceModel (defaults to empty Surface).
   * \param lambda The exponential distribution's lambda value of this PersistenceModel (defaults to 1).
   * \param count The number of observations used to create this model (i.e., our 'n'; defaults to 0).
   * \param last_seen The last time the item was seen (defaults to 0 time).
   */
  PersistenceModel(const world::Item &item = world::Item(), const world::Surface &surface = world::Surface(),
      const double lambda = 1, const uint32_t count = 0, const ros::Time &last_seen = ros::Time(0));

  /*!
   * \brief Item value accessor (immutable).
   *
   * Get the Item value of this PersistenceModel.
   *
   * \return The Item value of this PersistenceModel.
   */
  const world::Item &getItem() const;

  /*!
   * \brief Item value accessor.
   *
   * Get the Item value of this PersistenceModel.
   *
   * \return The Item value of this PersistenceModel.
   */
  world::Item &getItem();

  /*!
   * \brief Item value mutator.
   *
   * Set the Item value of this PersistenceModel.
   *
   * \param item The new Item value of this PersistenceModel.
   */
  void setItem(const world::Item &item);

  /*!
   * \brief Surface value accessor (immutable).
   *
   * Get the Surface value of this PersistenceModel.
   *
   * \return The Surface value of this PersistenceModel.
   */
  const world::Surface &getSurface() const;

  /*!
   * \brief Surface value accessor.
   *
   * Get the Surface value of this PersistenceModel.
   *
   * \return The Surface value of this PersistenceModel.
   */
  world::Surface &getSurface();

  /*!
   * \brief Surface value mutator.
   *
   * Set the Surface value of this PersistenceModel.
   *
   * \param surface The new Surface value of this PersistenceModel.
   */
  void setSurface(const world::Surface &surface);

  /*!
   * \brief Lambda value accessor.
   *
   * Get the lambda value of this PersistenceModel.
   *
   * \return The lambda value of this PersistenceModel.
   */
  double getLambda() const;

  /*!
   * \brief Lambda value mutator.
   *
   * Set the lambda value of this PersistenceModel.
   *
   * \param lambda The new lambda value of this PersistenceModel.
   */
  void setLambda(const double lambda);

  /*!
   * \brief Expected persistence value accessor.
   *
   * Get the expected persistence duration value of this PersistenceModel.
   *
   * \return The expected persistence duration value of this PersistenceModel.
   */
  ros::Duration getExpectedPersistence() const;

  /*!
   * \brief Probability Item still exists.
   *
   * Get probability the Item still exists on the Surface based on the given time.
   *
   * \param time The time to check the probability for (defaults to now).
   * \return The probability the Item still exists on the Surface based on the given time.
   */
  double getProbabilityItemStillExists(const ros::Time &time = ros::Time::now()) const;

  /*!
   * \brief Probability Item was removed.
   *
   * Get probability the Item was removed from the Surface based on the given time.
   *
   * \param time The time to check the probability for (defaults to now).
   * \return The probability the Item was removed from the Surface based on the given time.
   */
  double getProbabilityItemRemoved(const ros::Time &time = ros::Time::now()) const;

  /*!
   * \brief Count value accessor.
   *
   * Get the count value of this PersistenceModel.
   *
   * \return The count value of this PersistenceModel.
   */
  uint32_t getCount() const;

  /*!
   * \brief Count value mutator.
   *
   * Set the count value of this PersistenceModel.
   *
   * \param count The new count value of this PersistenceModel.
   */
  void setCount(const uint32_t count);

  /*!
   * \brief Last seen time value accessor (immutable).
   *
   * Get the time value of this PersistenceModel.
   *
   * \return The last seen time value of this PersistenceModel.
   */
  const ros::Time &getLastSeen() const;

  /*!
   * \brief Last seen time value accessor.
   *
   * Get the last seen time value of this PersistenceModel.
   *
   * \return The last seen time value of this PersistenceModel.
   */
  ros::Time &getLastSeen();

  /*!
   * \brief Last seen time value mutator.
   *
   * Set the time value of this PersistenceModel.
   *
   * \param last_seen The new last seen time value of this PersistenceModel.
   */
  void setLastSeen(const ros::Time &last_seen);

private:
  /*! The Item value for the model. */
  world::Item item_;
  /*! The Surface value for the model. */
  world::Surface surface_;
  /*! The exponential distribution. */
  boost::math::exponential_distribution<> exponential_;
  /*! The number of observations used to create this model (i.e., our 'n'). */
  uint32_t count_;
  /*! The last seen time the Item was observed on the Surface. */
  ros::Time last_seen_;
};

}
}
}
}

#endif
