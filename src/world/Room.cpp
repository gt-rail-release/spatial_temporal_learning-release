/*!
 * \file Room.cpp
 * \brief Room configuration information.
 *
 * A room consists of a series of surfaces.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

// worldlib
#include "worldlib/world/Room.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::geometry;
using namespace rail::spatial_temporal_learning::worldlib::world;

Room::Room(const string &name, const string &frame_id, const Pose &pose, const double width, const double depth,
    const double height) : Object(name, frame_id, pose, width, depth, height)
{
}

const vector<Surface> &Room::getSurfaces() const
{
  return surfaces_;
}

vector<Surface> &Room::getSurfaces()
{
  return surfaces_;
}

size_t Room::getNumSurfaces() const
{
  return surfaces_.size();
}

const Surface &Room::getSurface(const size_t index) const
{
  // check the index value first
  if (index < surfaces_.size())
  {
    return surfaces_[index];
  } else
  {
    throw out_of_range("Room::getSurface : Surface index does not exist.");
  }
}

Surface &Room::getSurface(const size_t index)
{
  // check the index value first
  if (index < surfaces_.size())
  {
    return surfaces_[index];
  } else
  {
    throw out_of_range("Room::getSurface : Surface index does not exist.");
  }
}

void Room::addSurface(const Surface &room)
{
  surfaces_.push_back(room);
}

void Room::removeSurface(const size_t index)
{
  // check the index value first
  if (index < surfaces_.size())
  {
    surfaces_.erase(surfaces_.begin() + index);
  } else
  {
    throw out_of_range("Room::removeSurface : Surface index does not exist.");
  }
}

bool Room::surfaceExists(const string &name) const
{
  // check if we can find it and catch any exceptions
  try
  {
    this->findSurface(name);
    return true;
  } catch (out_of_range &e)
  {
    return false;
  }
}

const Surface &Room::findSurface(const string &name) const
{
  // check each surface
  for (size_t i = 0; i < surfaces_.size(); i++)
  {
    // perform a check
    if (surfaces_[i].checkName(name))
    {
      return surfaces_[i];
    }
  }
  // no match found
  throw out_of_range("Room::findSurface : Surface name does not exist.");
}

Surface &Room::findSurface(const string &name)
{
  // check each surface
  for (size_t i = 0; i < surfaces_.size(); i++)
  {
    // perform a check
    if (surfaces_[i].checkName(name))
    {
      return surfaces_[i];
    }
  }
  // no match found
  throw out_of_range("Room::findSurface : Surface name does not exist.");
}

const Surface &Room::findClosestSurface(const Position &position) const
{
  return surfaces_[this->findClosestSurfaceIndex(position)];
}

Surface &Room::findClosestSurface(const Position &position)
{
  return surfaces_[this->findClosestSurfaceIndex(position)];
}

size_t Room::findClosestSurfaceIndex(const Position &position) const
{
  if (surfaces_.empty())
  {
    throw out_of_range("Room::findClosestSurface : No surfaces exist.");
  } else
  {
    // go through each and check the distance
    double best = numeric_limits<double>::infinity();
    size_t best_index = 0;
    for (size_t i = 0; i < surfaces_.size(); i++)
    {
      // calculate distance
      double distance = position.distance(surfaces_[i].getPose().getPosition());
      if (distance < best)
      {
        best = distance;
        best_index = i;
      }
    }

    return best_index;
  }
}

