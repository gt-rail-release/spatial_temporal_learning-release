/*!
 * \file Room.h
 * \brief Room configuration information.
 *
 * A room consists of a series of surfaces.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_ROOM_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_ROOM_H_

// worldlib
#include "Object.h"
#include "Surface.h"
#include "../geometry/Position.h"

// C++ Standard Library
#include <vector>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace world
{

/*!
 * \class Room
 * \brief Room configuration information.
 *
 * A room consists of a series of surfaces.
 */
class Room : public Object
{
public:
  /*!
   * \brief Create a new Room.
   *
   * Create a new empty Room with the given name, frame ID, Pose (in reference to the World frame ID) and dimensions.
   *
   * \param name The name of the Room (defaults to the empty string).
   * \param frame_id The frame ID of the Room (defaults to the empty string).
   * \param pose The Pose of the Room with respect to the World (defaults to 0 Pose).
   * \param width The width of the Room (along the x-axis) (defaults to 0).
   * \param depth The width of the Room (along the y-axis) (defaults to 0).
   * \param height The height of the Room (along the z-axis) (defaults to 0).
   */
  Room(const std::string &name = "", const std::string &frame_id = "", const geometry::Pose &pose = geometry::Pose(),
      const double width = 0, const double depth = 0, const double height = 0);

  /*!
   * \brief Surfaces value accessor (immutable).
   *
   * Get the surfaces of this Room.
   *
   * \return The surfaces.
   */
  const std::vector<Surface> &getSurfaces() const;

  /*!
   * \brief Surfaces value accessor.
   *
   * Get the surfaces of this Room.
   *
   * \return The surfaces.
   */
  std::vector<Surface> &getSurfaces();

  /*!
   * \brief Surfaces size accessor.
   *
   * Get the number of surfaces of this Room.
   *
   * \return The number of surfaces of this Room.
   */
  size_t getNumSurfaces() const;

  /*!
   * \brief Surface value accessor (immutable).
   *
   * Get the Surface of this Room at the given index.
   *
   * \param i The index of the Surface to get.
   * \return The Surface at the given index.
   * \throws std::out_of_range Thrown if the Surface at the given index does not exist.
   */
  const Surface &getSurface(const size_t index) const;

  /*!
   * \brief Surface value accessor.
   *
   * Get the Surface of this Room at the given index.
   *
   * \param i The index of the Surface to get.
   * \return The Surface at the given index.
   * \throws std::out_of_range Thrown if the Surface at the given index does not exist.
   */
  Surface &getSurface(const size_t index);

  /*!
   * \brief Surface adder.
   *
   * Add the Surface to this Room.
   *
   * \param Surface The new Surface to add.
   */
  void addSurface(const Surface &surface);

  /*!
   * \brief Surface remover.
   *
   * Remove the Surface at the given index. An invalid index results in no effect.
   *
   * \param i The index of the Surface to remove.
   * \throws std::out_of_range Thrown if the Surface at the given index does not exist.
   */
  void removeSurface(const size_t index);

  /*!
   * \brief Check for the existence of a Surface.
   *
   * Check for the existence of a Surface in the Room. This will also check the aliases. Case is not important.
   *
   * \param name The name or alias of the Surface to find.
   * \throws std::out_of_range Thrown if no Surface with the given name exists.
   */
  bool surfaceExists(const std::string &name) const;

  /*!
   * \brief Surface finder (immutable).
   *
   * Find a Surface with the given name. This will also check the aliases. Case is not important. If multiple
   * Surface exist with the given name, the first Surface is returned.
   *
   * \param name The name or alias of the Surface to find.
   * \throws std::out_of_range Thrown if no Surface with the given name exists.
   */
  const Surface &findSurface(const std::string &name) const;

  /*!
   * \brief Surface finder.
   *
   * Find a Surface with the given name. This will also check the aliases. Case is not important. If multiple
   * Surface exist with the given name, the first Surface is returned.
   *
   * \param name The name or alias of the Surface to find.
   * \throws std::out_of_range Thrown if no Surface with the given name exists.
   */
  Surface &findSurface(const std::string &name);

  /*!
   * \brief Closest Surface finder (immutable).
   *
   * Find the closest surface to the given Position in the room's fixed frame.
   *
   * \param position The Position to find the closest Surface to (in the room's fixed frame).
   * \returns The closest Surface.
   * \throws std::out_of_range Thrown if no Surfaces exist in the Room.
   */
  const Surface &findClosestSurface(const geometry::Position &position) const;

  /*!
   * \brief Closest Surface finder.
   *
   * Find the closest surface to the given Position in the room's fixed frame.
   *
   * \param position The Position to find the closest Surface to (in the room's fixed frame).
   * \returns The closest Surface.
   * \throws std::out_of_range Thrown if no Surfaces exist in the Room.
   */
  Surface &findClosestSurface(const geometry::Position &position);

  /*!
   * \brief Closest Surface index finder.
   *
   * Find the closest Surface index to the given Position. If no Surface exists, an exception is thrown.
   *
   * \param position The position (in reference to the world's frame) to find the closest Surface to.
   * \return The index of the closest Surface.
   * \throws std::out_of_range Thrown if no Surface exists.
   */
  size_t findClosestSurfaceIndex(const geometry::Position &position) const;

private:
  /*! List of surfaces. */
  std::vector<Surface> surfaces_;
};

}
}
}
}

#endif
