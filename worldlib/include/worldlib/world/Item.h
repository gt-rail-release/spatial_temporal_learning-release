/*!
 * \file Item.h
 * \brief Item configuration information.
 *
 * An item represents a manipulable object within the world.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_ITEM_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_ITEM_H_

// worldlib
#include "Object.h"

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace world
{

/*!
 * \class Item
 * \brief Item configuration information.
 *
 * An item represents a manipulable object within the world.
 */
class Item : public Object
{
public:
  /*!
   * \brief Create a new Item.
   *
   * Create a new empty Item with the given name, frame ID, Pose (in reference to the surfaces' frame ID if it is known)
   * and dimensions.
   *
   * \param name The name of the Item (defaults to the empty string).
   * \param frame_id The frame ID of the Item (defaults to the empty string).
   * \param pose The Pose of the Item with respect to the Surface if known (defaults to 0 Pose).
   * \param width The width of the Item (along the x-axis) (defaults to 0).
   * \param depth The width of the Item (along the y-axis) (defaults to 0).
   * \param height The height of the Item (along the z-axis) (defaults to 0).
   */
  Item(const std::string &name = "", const std::string &frame_id = "", const geometry::Pose &pose = geometry::Pose(),
      const double width = 0, const double depth = 0, const double height = 0);
};

}
}
}
}

#endif
