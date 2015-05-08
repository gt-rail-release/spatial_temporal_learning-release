/*!
 * \file Observation.cpp
 * \brief World observation configuration information.
 *
 * An observation consists of seeing a given item on a given surface at a give pose.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 24, 2015
 */

// worldlib
#include "worldlib/world/Observation.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::geometry;
using namespace rail::spatial_temporal_learning::worldlib::world;

Observation::Observation(const Item &item, const Surface &surface, const Pose &pose, const ros::Time &time,
    const ros::Time &removed_estimate, const ros::Time &removed_observed)
    : item_(item), surface_(surface), pose_(pose), time_(time), removed_estimate_(removed_estimate),
      removed_observed_(removed_observed)
{
}

const Item &Observation::getItem() const
{
  return item_;
}

Item &Observation::getItem()
{
  return item_;
}

void Observation::setItem(const Item &item)
{
  item_ = item;
}

const Surface &Observation::getSurface() const
{
  return surface_;
}

Surface &Observation::getSurface()
{
  return surface_;
}

void Observation::setSurface(const Surface &surface)
{
  surface_ = surface;
}

const PlacementSurface &Observation::getPlacementSurface() const
{
  return surface_.findClosestPlacementSurface(pose_.getPosition());
}

const Pose &Observation::getPose() const
{
  return pose_;
}

Pose &Observation::getPose()
{
  return pose_;
}

void Observation::setPose(const Pose &pose)
{
  pose_ = pose;
}

const ros::Time &Observation::getTime() const
{
  return time_;
}

ros::Time &Observation::getTime()
{
  return time_;
}

void Observation::setTime(const ros::Time &time)
{
  time_ = time;
}

const ros::Time &Observation::getRemovedEstimate() const
{
  return removed_estimate_;
}

ros::Time &Observation::getRemovedEstimate()
{
  return removed_estimate_;
}

void Observation::setRemovedEstimate(const ros::Time &removed_estimate)
{
  removed_estimate_ = removed_estimate;
}

const ros::Time &Observation::getRemovedObserved() const
{
  return removed_observed_;
}

ros::Time &Observation::getRemovedObserved()
{
  return removed_observed_;
}

void Observation::setRemovedObserved(const ros::Time &removed_observed)
{
  removed_observed_ = removed_observed;
}
