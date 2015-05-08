/*!
 * \file Surface.cpp
 * \brief Surface configuration information.
 *
 * A surface consists of a name with associated placement frames and points of interest.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

// worldlib
#include "worldlib/world/Surface.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::geometry;
using namespace rail::spatial_temporal_learning::worldlib::world;

Surface::Surface(const string &name, const string &frame_id, const geometry::Pose &pose, const double width,
    const double depth, const double height) : Object(name, frame_id, pose, width, depth, height)
{
}

const vector<PlacementSurface> &Surface::getPlacementSurfaces() const
{
  return placement_surfaces_;
}

vector<PlacementSurface> &Surface::getPlacementSurfaces()
{
  return placement_surfaces_;
}

size_t Surface::getNumPlacementSurfaces() const
{
  return placement_surfaces_.size();
}

const PlacementSurface &Surface::getPlacementSurface(const size_t index) const
{
  // check the index value first
  if (index < placement_surfaces_.size())
  {
    return placement_surfaces_[index];
  } else
  {
    throw out_of_range("Surface::getPlacementSurface : Placement surface index does not exist.");
  }
}

PlacementSurface &Surface::getPlacementSurface(const size_t index)
{
  // check the index value first
  if (index < placement_surfaces_.size())
  {
    return placement_surfaces_[index];
  } else
  {
    throw out_of_range("Surface::getPlacementSurface : Placement surface index does not exist.");
  }
}

void Surface::addPlacementSurface(const PlacementSurface &placement_surface)
{
  placement_surfaces_.push_back(placement_surface);
}

void Surface::removePlacementSurface(const size_t index)
{
  // check the index value first
  if (index < placement_surfaces_.size())
  {
    placement_surfaces_.erase(placement_surfaces_.begin() + index);
  } else
  {
    throw out_of_range("Surface::removePlacementSurface : Placement surface index does not exist.");
  }
}

bool Surface::placementSurfaceExists(const string &name) const
{
  // check if we can find it and catch any exceptions
  try
  {
    this->findPlacementSurface(name);
    return true;
  } catch (out_of_range &e)
  {
    return false;
  }
}

const PlacementSurface &Surface::findPlacementSurface(const string &name) const
{
  // check each surface
  for (size_t i = 0; i < placement_surfaces_.size(); i++)
  {
    // perform a check
    if (placement_surfaces_[i].checkName(name))
    {
      return placement_surfaces_[i];
    }
  }
  // no match found
  throw out_of_range("Surface::findPlacementSurface : Placement surface name does not exist.");
}

PlacementSurface &Surface::findPlacementSurface(const string &name)
{
  // check each surface
  for (size_t i = 0; i < placement_surfaces_.size(); i++)
  {
    // perform a check
    if (placement_surfaces_[i].checkName(name))
    {
      return placement_surfaces_[i];
    }
  }
  // no match found
  throw out_of_range("Surface::findPlacementSurface : Placement surface name does not exist.");
}

const PlacementSurface &Surface::findClosestPlacementSurface(const Position &position) const
{
  return placement_surfaces_[this->findClosestPlacementSurfaceIndex(position)];
}

PlacementSurface &Surface::findClosestPlacementSurface(const Position &position)
{
  return placement_surfaces_[this->findClosestPlacementSurfaceIndex(position)];
}

size_t Surface::findClosestPlacementSurfaceIndex(const Position &position) const
{
  if (placement_surfaces_.empty())
  {
    throw out_of_range("Surface::findClosestPlacementSurface : No placement surfaces exist.");
  } else
  {
    // go through each and check the distance
    double best = numeric_limits<double>::infinity();
    size_t best_index = 0;
    for (size_t i = 0; i < placement_surfaces_.size(); i++)
    {
      // calculate distance
      double distance = position.distance(placement_surfaces_[i].getPose().getPosition());
      if (distance < best)
      {
        best = distance;
        best_index = i;
      }
    }

    return best_index;
  }
}

const vector<PointOfInterest> &Surface::getPointsOfInterest() const
{
  return pois_;
}

vector<PointOfInterest> &Surface::getPointsOfInterest()
{
  return pois_;
}

size_t Surface::getNumPointsOfInterest() const
{
  return pois_.size();
}

const PointOfInterest &Surface::getPointOfInterest(const size_t index) const
{
  // check the index value first
  if (index < pois_.size())
  {
    return pois_[index];
  } else
  {
    throw out_of_range("Surface::getPointOfInterest : Point of interest index does not exist.");
  }
}

PointOfInterest &Surface::getPointOfInterest(const size_t index)
{
  // check the index value first
  if (index < pois_.size())
  {
    return pois_[index];
  } else
  {
    throw out_of_range("Surface::getPointOfInterest : Point of interest index does not exist.");
  }
}

void Surface::addPointOfInterest(const PointOfInterest &poi)
{
  pois_.push_back(poi);
}

void Surface::removePointOfInterest(const size_t index)
{
  // check the index value first
  if (index < pois_.size())
  {
    pois_.erase(pois_.begin() + index);
  } else
  {
    throw out_of_range("Surface::removePointOfInterest : Point of interest index does not exist.");
  }
}

bool Surface::pointOfInterestExists(const string &name) const
{
  // check if we can find it and catch any exceptions
  try
  {
    this->findPointOfInterest(name);
    return true;
  } catch (out_of_range &e)
  {
    return false;
  }
}

const PointOfInterest &Surface::findPointOfInterest(const string &name) const
{
  // check each surface
  for (size_t i = 0; i < pois_.size(); i++)
  {
    // perform a check
    if (pois_[i].checkName(name))
    {
      return pois_[i];
    }
  }
  // no match found
  throw out_of_range("Surface::findPointOfInterest : Point of interest name does not exist.");
}


PointOfInterest &Surface::findPointOfInterest(const string &name)
{
  // check each surface
  for (size_t i = 0; i < pois_.size(); i++)
  {
    // perform a check
    if (pois_[i].checkName(name))
    {
      return pois_[i];
    }
  }
  // no match found
  throw out_of_range("Surface::findPointOfInterest : Point of interest name does not exist.");
}
