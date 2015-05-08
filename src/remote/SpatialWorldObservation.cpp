/*!
 * \file SqlObservation.cpp
 * \brief Observation information from an SQL entity in the spatial world database.
 *
 * An observation consists of seeing a given item on a given surface at a give pose. This class corresponds directly
 * to an entity in the spatial world database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 24, 2015
 */

// worldlib
#include "worldlib/remote/SpatialWorldObservation.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::geometry;
using namespace rail::spatial_temporal_learning::worldlib::remote;

SpatialWorldObservation::SpatialWorldObservation(const uint32_t id, const string &item_name, const string &surface_name,
    const string &surface_frame_id, const Pose &pose, const ros::Time &time, const ros::Time &removed_estimate,
    const ros::Time &removed_observed)
    : SqlEntity(id),
      item_name_(item_name), surface_name_(surface_name), surface_frame_id_(surface_frame_id), pose_(pose), time_(time),
      removed_estimate_(removed_estimate), removed_observed_(removed_observed)
{
}

const string &SpatialWorldObservation::getItemName() const
{
  return item_name_;
}

void SpatialWorldObservation::setItemName(const string &item_name)
{
  item_name_ = item_name;
}

const string &SpatialWorldObservation::getSurfaceName() const
{
  return surface_name_;
}

void SpatialWorldObservation::setSurfaceName(const string &surface_name)
{
  surface_name_ = surface_name;
}

const string &SpatialWorldObservation::getSurfaceFrameID() const
{
  return surface_frame_id_;
}

void SpatialWorldObservation::setSurfaceFrameID(const string &surface_frame_id)
{
  surface_frame_id_ = surface_frame_id;
}

const Pose &SpatialWorldObservation::getPose() const
{
  return pose_;
}

Pose &SpatialWorldObservation::getPose()
{
  return pose_;
}

void SpatialWorldObservation::setPose(const Pose &pose)
{
  pose_ = pose;
}

const ros::Time &SpatialWorldObservation::getTime() const
{
  return time_;
}

ros::Time &SpatialWorldObservation::getTime()
{
  return time_;
}

void SpatialWorldObservation::setTime(const ros::Time &time)
{
  time_ = time;
}

const ros::Time &SpatialWorldObservation::getRemovedEstimate() const
{
  return removed_estimate_;
}

ros::Time &SpatialWorldObservation::getRemovedEstimate()
{
  return removed_estimate_;
}

void SpatialWorldObservation::setRemovedEstimate(const ros::Time &removed_estimate)
{
  removed_estimate_ = removed_estimate;
}

const ros::Time &SpatialWorldObservation::getRemovedObserved() const
{
  return removed_observed_;
}

ros::Time &SpatialWorldObservation::getRemovedObserved()
{
  return removed_observed_;
}

void SpatialWorldObservation::setRemovedObserved(const ros::Time &removed_observed)
{
  removed_observed_ = removed_observed;
}
