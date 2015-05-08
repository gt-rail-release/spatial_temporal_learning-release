/*!
 * \file World.h
 * \brief World configuration information.
 *
 * A world consists of a series of rooms, surfaces, and items. Surfaces can have points of interest as well.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_WORLD_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_WORLD_H_

// worldlib
#include "Item.h"
#include "Room.h"

// ROS
#include <tf2_ros/buffer.h>

// YAML
#include <yaml-cpp/yaml.h>

// C++ Standard Library
#include <string>
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
 * \class World
 * \brief World configuration information.
 *
 * A world consists of a series of rooms, surfaces, and items. Surfaces can have points of interest as well.
 */
class World
{
public:
  /*!
   * \brief Create a new World.
   *
   * Creates a new empty World with an optional fixed frame ID.
   *
   * \param fixed_frame_id The fixed frame ID of the World (defaults to the empty string).
   */
  World(const std::string &fixed_frame_id = "");

  /*!
   * \brief Load configuration data from a YAML file.
   *
   * Load world configuration information from the given YAML file. This will attempt to load Pose data from a TF
   * client and thus should only be run on a live system.
   *
   * \param file The name of the YAML file to load.
   * \return True if configuration data was successfully loaded.
   */
  bool loadFromYaml(const std::string &file);

#ifdef YAMLCPP_GT_0_5_0
  /*!
   * \brief Parse basic Object info from a YAML node.
   *
   * Parse the given YAML node as an Object and return that Object. This assumes no Pose data is given.
   *
   * \param object The YAML node to parse.
   * \return The Object parsed.
   */
  Object parseObject(const YAML::Node &object) const;
#endif

  /*!
   * \brief Fixed frame ID accessor.
   *
   * Get the fixed frame ID this World.
   *
   * \return The fixed frame ID of this World.
   */
  const std::string &getFixedFrameID() const;

  /*!
   * \brief Fixed frame ID mutator.
   *
   * Set the fixed frame ID this World to the given value.
   *
   * \param The new fixed frame ID value of this World.
   */
  void setFixedFrameID(const std::string &fixed_frame_id);

  /*!
   * \brief Rooms value accessor (immutable).
   *
   * Get the rooms of this World.
   *
   * \return The rooms.
   */
  const std::vector<Room> &getRooms() const;

  /*!
   * \brief Rooms value accessor.
   *
   * Get the rooms of this World.
   *
   * \return The rooms.
   */
  std::vector<Room> &getRooms();

  /*!
   * \brief Rooms size accessor.
   *
   * Get the number of rooms of this World.
   *
   * \return The number of rooms of this World.
   */
  size_t getNumRooms() const;

  /*!
   * \brief Room value accessor (immutable).
   *
   * Get the Room of this World at the given index.
   *
   * \param i The index of the Room to get.
   * \return The Room at the given index.
   * \throws std::out_of_range Thrown if the Room at the given index does not exist.
   */
  const Room &getRoom(const size_t index) const;

  /*!
   * \brief Room value accessor.
   *
   * Get the Room of this World at the given index.
   *
   * \param i The index of the Room to get.
   * \return The Room at the given index.
   * \throws std::out_of_range Thrown if the Room at the given index does not exist.
   */
  Room &getRoom(const size_t index);

  /*!
   * \brief Room adder.
   *
   * Add the Room to this World.
   *
   * \param room The new Room to add.
   */
  void addRoom(const Room &room);

  /*!
   * \brief Room remover.
   *
   * Remove the Room at the given index. An invalid index results in no effect.
   *
   * \param i The index of the Room pose to remove.
   * \throws std::out_of_range Thrown if the Room at the given index does not exist.
   */
  void removeRoom(const size_t index);

  /*!
   * \brief Check for the existence of a Room.
   *
   * Check for the existence of a Room in the World. This will also check the aliases. Case is not important.
   *
   * \param name The name or alias of the Room to find.
   * \throws std::out_of_range Thrown if no Room with the given name exists.
   */
  bool roomExists(const std::string &name) const;

  /*!
   * \brief Room finder (immutable).
   *
   * Find a Room with the given name. This will also check the aliases. Case is not important. If multiple rooms exist
   * with the given name, the first Room is returned.
   *
   * \param name The name or alias of the Room to find.
   * \throws std::out_of_range Thrown if no PointOfInterest with the given name exists.
   */
  const Room &findRoom(const std::string &name) const;

  /*!
   * \brief Room finder.
   *
   * Find a Room with the given name. This will also check the aliases. Case is not important. If multiple rooms exist
   * with the given name, the first Room is returned.
   *
   * \param name The name or alias of the Room to find.
   * \throws std::out_of_range Thrown if no Room with the given name exists.
   */
  Room &findRoom(const std::string &name);

  /*!
   * \brief Items value accessor (immutable).
   *
   * Get the items of this World.
   *
   * \return The items.
   */
  const std::vector<Item> &getItems() const;

  /*!
   * \brief Items value accessor (immutable).
   *
   * Get the items of this World.
   *
   * \return The items.
   */
  std::vector<Item> &getItems();

  /*!
   * \brief Items size accessor.
   *
   * Get the number of items of this World.
   *
   * \return The number of items of this World.
   */
  size_t getNumItems() const;

  /*!
   * \brief Item value accessor (immutable).
   *
   * Get the Item of this World at the given index.
   *
   * \param i The index of the Item to get.
   * \return The Item at the given index.
   * \throws std::out_of_range Thrown if the Item at the given index does not exist.
   */
  const Item &getItem(const size_t index) const;

  /*!
   * \brief Item value accessor.
   *
   * Get the Item of this World at the given index.
   *
   * \param i The index of the Item to get.
   * \return The Item at the given index.
   * \throws std::out_of_range Thrown if the Item at the given index does not exist.
   */
  Item &getItem(const size_t index);

  /*!
   * \brief Item adder.
   *
   * Add the Item to this World.
   *
   * \param item The new Item to add.
   */
  void addItem(const Item &item);

  /*!
   * \brief Item remover.
   *
   * Remove the Item at the given index. An invalid index results in no effect.
   *
   * \param i The index of the Item pose to remove.
   * \throws std::out_of_range Thrown if the Item at the given index does not exist.
   */
  void removeItem(const size_t index);

  /*!
   * \brief Check for the existence of an Item.
   *
   * Check for the existence of an Item in the World. This will also check the aliases. Case is not important.
   *
   * \param name The name or alias of the Item to find.
   * \returns If the Item exists.
   * \throws std::out_of_range Thrown if no Room with the given name exists.
   */
  bool itemExists(const std::string &name) const;

  /*!
   * \brief Item finder (immutable).
   *
   * Find an Item with the given name. This will also check the aliases. Case is not important. If multiple items exist
   * with the given name, the first Item is returned.
   *
   * \param name The name or alias of the Item to find.
   * \returns The Item with the given name.
   * \throws std::out_of_range Thrown if no Item with the given name exists.
   */
  const Item &findItem(const std::string &name) const;

  /*!
   * \brief Item finder.
   *
   * Find an Item with the given name. This will also check the aliases. Case is not important. If multiple items exist
   * with the given name, the first Item is returned.
   *
   * \param name The name or alias of the Item to find.
   * \returns The Item with the given name.
   * \throws std::out_of_range Thrown if no Item with the given name exists.
   */
  Item &findItem(const std::string &name);

  /*!
   * \brief Closest Surface finder.
   *
   * Find the closest surface to the given Position in the world fixed frame.
   *
   * \param position The Position to find the closest Surface to (in the world's fixed frame).
   * \param room_index Will be set to the Room index in the World.
   * \param surface_index Will be set to the Surface index in the Room.
   * \throws std::out_of_range Thrown if no Rooms or Surfaces exist in the World.
   */
  void findClosestSurface(const geometry::Position &position, size_t &room_index, size_t &surface_index) const;

  /*!
   * \brief PlacementSurface finder.
   *
   * Find the PlacementSurface (if any) that the Position is on. This will checking the bounding range of each
   * Surface and find the closest PlacementSurface the Position is on. Since it is likely a Position will not be on a
   * Placement surface (e.g., from noise or floating objects), this method returns false instead of throwing an
   * exception.
   *
   * \param position The Position to find the PlacemetSurface to (in the world's fixed frame).
   * \param room_index Will be set to the Room index in the World.
   * \param surface_index Will be set to the Surface index in the Room.
   * \param placement_surface_index Will be set to the PlacemetSurface index in the Surface.
   * \return If a valid PlacementSurface was found.
   */
  bool findPlacementSurface(const geometry::Position &position, size_t &room_index, size_t &surface_index,
      size_t &placement_surface_index) const;

private:
  /*! The fixed frame of the world. */
  std::string fixed_frame_id_;
  /*! Room information for the world. */
  std::vector<Room> rooms_;
  /*! Item information for the world. */
  std::vector<Item> items_;
};

}
}
}
}

#endif
