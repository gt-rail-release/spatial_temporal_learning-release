/*!
 * \file SqlEntity.h
 * \brief Common SQL entity information.
 *
 * A base class for all SQL entities.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 24, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_SQL_ENTITY_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_SQL_ENTITY_H_

// C++ Standard Library
#include <stdint.h>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace remote
{

/*!
 * \class SqlEntity
 * \brief Common SQL entity information.
 *
 * A base class for all SQL entities.
 */
class SqlEntity
{
public:
  /*!
   * \brief Create a new SqlEntity.
   *
   * Create a new SqlEntity with the given ID information.
   *
   * \param id The ID of the database entity.
   */
  SqlEntity(const uint32_t id);

  /*!
   * \brief ID value accessor.
   *
   * Get the ID value of this SqlEntity.
   *
   * \return The ID value of this SqlEntity.
   */
  uint32_t getID() const;

  /*!
   * \brief ID value mutator.
   *
   * Set the ID value of this SpatialWorldObservation.
   *
   * \param id The new id value of this SpatialWorldObservation.
   */
  void setID(const uint32_t id);

private:
  /*! The ID of the entity. */
  uint32_t id_;
};

}
}
}
}

#endif
