/*!
 * \file Placement.h
 * \brief World placement configuration information.
 *
 * A placement consists of an Item with respect to some Object at a given Pose on a Surface.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 25, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_PLACEMENT_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_PLACEMENT_H_

// worldlib
#include "Item.h"
#include "Surface.h"

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace world
{

/*!
 * \class Placement
 * \brief World placement configuration information.
 *
 * A placement consists of an Item with respect to some Object at a given Pose on a Surface.
 */
class Placement
{
public:
  /*!
   * \brief Create a new Placement.
   *
   * Create a new Placement with the given Item (with a Pose in reference to the Object), the reference Object, and
   * the Surface it was placed on.
   *
   * \param item The Item seen in this Placement (the Pose of the Item should be in reference to the Object).
   * \param object The Object the Item was placed in reference to.
   * \param pose The Pose of the Item in with respect to the Object.
   */
  Placement(const Item &item, const Object &object, const Surface &pose);

  /*!
   * \brief Item value accessor (immutable).
   *
   * Get the Item value of this Placement.
   *
   * \return The Item value of this Placement.
   */
  const Item &getItem() const;

  /*!
   * \brief Item value accessor.
   *
   * Get the Item value of this Placement.
   *
   * \return The Item value of this Placement.
   */
  Item &getItem();

  /*!
   * \brief Item value mutator.
   *
   * Set the Item value of this Placement.
   *
   * \param item The new Item value of this Placement.
   */
  void setItem(const Item &item);

  /*!
   * \brief Object value accessor (immutable).
   *
   * Get the Object value of this Placement.
   *
   * \return The Object value of this Placement.
   */
  const Object &getObject() const;

  /*!
   * \brief Object value accessor.
   *
   * Get the Object value of this Placement.
   *
   * \return The Object value of this Placement.
   */
  Object &getObject();

  /*!
   * \brief Object value mutator.
   *
   * Set the Object value of this Placement.
   *
   * \param object The new Object value of this Placement.
   */
  void setObject(const Object &object);

  /*!
   * \brief Surface value accessor (immutable).
   *
   * Get the Surface value of this Placement.
   *
   * \return The Surface value of this Placement.
   */
  const Surface &getSurface() const;

  /*!
   * \brief Surface value accessor.
   *
   * Get the Surface value of this Placement.
   *
   * \return The Surface value of this Placement.
   */
  Surface &getSurface();

  /*!
   * \brief Surface value mutator.
   *
   * Set the Surface value of this Placement.
   *
   * \param surface The new Surface value of this Placement.
   */
  void setSurface(const Surface &surface);

private:
  /*! The Item in this Placement. */
  Item item_;
  /*! The Object in this Placement. */
  Object object_;
  /*! The Surface in this Placement. */
  Surface surface_;
};

}
}
}
}

#endif
