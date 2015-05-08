/*!
 * \file GeoLifeEntry.cpp
 * \brief The GeoLife entry contains a timestamp with latitude and longitude information.
 *
 * An  GeoLife entry simply contains latitude and longitude information with an associated ROS timestamp.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 30, 2015
 */

// World Item Search
#include "world_item_search/GeoLifeEntry.h"

using namespace rail::spatial_temporal_learning;

GeoLifeEntry::GeoLifeEntry(const double latitude, const double longitude, const ros::Time &time) : time_(time)
{
  latitude_ = latitude;
  longitude_ = longitude;
}

double GeoLifeEntry::getLatitude() const
{
  return latitude_;
}

void GeoLifeEntry::setLatitude(const double latitude)
{
  latitude_ = latitude;
}

double GeoLifeEntry::getLongitude() const
{
  return longitude_;
}

void GeoLifeEntry::setLongitude(const double longitude)
{
  longitude_ = longitude;
}

const ros::Time &GeoLifeEntry::getTime() const
{
  return time_;
}

ros::Time &GeoLifeEntry::getTime()
{
  return time_;
}

void GeoLifeEntry::setTime(const ros::Time &time)
{
  time_ = time;
}
