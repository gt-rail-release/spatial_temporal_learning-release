/*!
 * \file SqlEntity.cpp
 * \brief Common SQL entity information.
 *
 * A base class for all SQL entities.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 24, 2015
 */

// worldlib
#include "worldlib/remote/SqlEntity.h"

using namespace rail::spatial_temporal_learning::worldlib::remote;

SqlEntity::SqlEntity(const uint32_t id)
{
  id_ = id;
}

uint32_t SqlEntity::getID() const
{
  return id_;
}

void SqlEntity::setID(const uint32_t id)
{
  id_ = id;
}
