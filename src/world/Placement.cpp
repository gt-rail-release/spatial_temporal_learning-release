/*!
 * \file Placement.cpp
 * \brief World placement configuration information.
 *
 * A placement consists of an Item with respect to some Object at a given Pose on a Surface.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 25, 2015
 */

// worldlib
#include "worldlib/world/Placement.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::world;

Placement::Placement(const Item &item, const Object &object, const Surface &surface)
    : item_(item), object_(object), surface_(surface)
{
}

const Item &Placement::getItem() const
{
  return item_;
}

Item &Placement::getItem()
{
  return item_;
}

void Placement::setItem(const Item &item)
{
  item_ = item;
}

const Object &Placement::getObject() const
{
  return object_;
}

Object &Placement::getObject()
{
  return object_;
}

void Placement::setObject(const Object &object)
{
  object_ = object;
}

const Surface &Placement::getSurface() const
{
  return surface_;
}

Surface &Placement::getSurface()
{
  return surface_;
}

void Placement::setSurface(const Surface &surface)
{
  surface_ = surface;
}
