/*!
 * \file PersistenceModel.cpp
 * \brief Persistence model information.
 *
 * A persistence model contains information about the estimated time an item will disappear from the surface.
 * Persistence models are in the granularity of hours.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 28, 2015
 */

// worldlib
#include "worldlib/model/PersistenceModel.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::model;
using namespace rail::spatial_temporal_learning::worldlib::world;

PersistenceModel::PersistenceModel(const Item &item, const Surface &surface, const double lambda, const uint32_t count,
    const ros::Time &last_seen) : item_(item), surface_(surface), last_seen_(last_seen), exponential_(lambda)
{
  count_ = count;
}

const Item &PersistenceModel::getItem() const
{
  return item_;
}

Item &PersistenceModel::getItem()
{
  return item_;
}

void PersistenceModel::setItem(const Item &item)
{
  item_ = item;
}

const Surface &PersistenceModel::getSurface() const
{
  return surface_;
}

Surface &PersistenceModel::getSurface()
{
  return surface_;
}

void PersistenceModel::setSurface(const Surface &surface)
{
  surface_ = surface;
}

double PersistenceModel::getLambda() const
{
  return exponential_.lambda();
}

void PersistenceModel::setLambda(const double lambda)
{
  exponential_ = boost::math::exponential_distribution<>(lambda);
}

ros::Duration PersistenceModel::getExpectedPersistence() const
{
  // get the time in hours and convert to seconds
  double hours = boost::math::mean(exponential_);
  double seconds = hours * 3600;
  return ros::Duration(seconds);
}

double PersistenceModel::getProbabilityItemStillExists(const ros::Time &time) const
{
  // need the complement of the CDF (probability the event did not occur yet)
  return 1.0 - this->getProbabilityItemRemoved(time);
}

double PersistenceModel::getProbabilityItemRemoved(const ros::Time &time) const
{
  // calculate the time from when we last saw it
  ros::Duration duration = time - last_seen_;
  // convert to hours
  double hours = duration.toSec() / 3600;
  // calculate the CDF (probability the event did occur)
  return boost::math::cdf(exponential_, abs(hours));
}

uint32_t PersistenceModel::getCount() const
{
  return count_;
}

void PersistenceModel::setCount(const uint32_t count)
{
  count_ = count;
}

const ros::Time &PersistenceModel::getLastSeen() const
{
  return last_seen_;
}

ros::Time &PersistenceModel::getLastSeen()
{
  return last_seen_;
}

void PersistenceModel::setLastSeen(const ros::Time &last_seen)
{
  last_seen_ = last_seen;
}
