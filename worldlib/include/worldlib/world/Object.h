/*!
 * \file Object.h
 * \brief Object configuration information.
 *
 * An object is an abstract entity in the world. Objects can be mutable entities (e.g., manipulable items) or
 * immutable entities (e.g., rooms, surfaces, and furniture). Objects can have an array of associated aliases
 * associated with their main name.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_OBJECT_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_OBJECT_H_

// worldlib
#include "../geometry/Pose.h"

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
 * \class Object
 * \brief Object configuration information.
 *
 * An object is an abstract entity in the world. Objects can be mutable entities (e.g., manipulable items) or
 * immutable entities (e.g., surfaces and furniture). Objects can have an array of associated aliases associated with
 * their main name.
 */
class Object
{
public:
  /*!
   * \brief Create a new Object.
   *
   * Create a new Object with the given name, frame ID, Pose (in reference to the parent's frame ID), and
   * dimensions.
   *
   * \param name The name of the Object (defaults to the empty string).
   * \param frame_id The frame ID of the Object (defaults to the empty string).
   * \param pose The Pose of the Object with respect to the parent frame (defaults to 0 Pose).
   * \param width The width of the Object (along the x-axis) (defaults to 0).
   * \param depth The width of the Object (along the y-axis) (defaults to 0).
   * \param height The height of the Object (along the z-axis) (defaults to 0).
   */
  Object(const std::string &name = "", const std::string &frame_id = "", const geometry::Pose &pose = geometry::Pose(),
      const double width = 0, const double depth = 0, const double height = 0);

  /*!
   * \brief Name value accessor.
   *
   * Get the name value of this Object.
   *
   * \return The name value of this Object.
   */
  const std::string &getName() const;

  /*!
   * \brief Name value mutator.
   *
   * Set the name value of this Object.
   *
   * \param name The new name value of this Object.
   */
  void setName(const std::string &name);

  /*!
   * \brief Frame ID value accessor.
   *
   * Get the frame ID value of this Object.
   *
   * \return The frame ID value of this Object.
   */
  const std::string &getFrameID() const;

  /*!
   * \brief Frame ID value mutator.
   *
   * Set the frame ID value of this Object.
   *
   * \param frame_id The new frame ID value of this Object.
   */
  void setFrameID(const std::string &frame_id);

  /*!
   * \brief Pose value accessor (immutable).
   *
   * Get the Pose value of this Object (with respect to its parents frame).
   *
   * \return The Pose value of this Object.
   */
  const geometry::Pose &getPose() const;

  /*!
   * \brief Pose value accessor.
   *
   * Get the Pose value of this Object (with respect to its parents frame).
   *
   * \return The Pose value of this Object.
   */
  geometry::Pose &getPose();

  /*!
   * \brief Pose value mutator.
   *
   * Set the Pose value of this Object.
   *
   * \param pose The new Pose value of this Object.
   */
  void setPose(geometry::Pose &pose);

  /*!
   * \brief Width value accessor.
   *
   * Get the width value of this Object.
   *
   * \return The width value of this Object.
   */
  double getWidth() const;

  /*!
   * \brief Width value mutator.
   *
   * Set the width value of this Object.
   *
   * \param width The new width value of this Object.
   */
  void setWidth(const double width);

  /*!
   * \brief Depth value accessor.
   *
   * Get the depth value of this Object.
   *
   * \return The depth value of this Object.
   */
  double getDepth() const;

  /*!
   * \brief Depth value mutator.
   *
   * Set the depth value of this Object.
   *
   * \param depth The new depth value of this Object.
   */
  void setDepth(const double depth);

  /*!
   * \brief Height value accessor.
   *
   * Get the height value of this Object.
   *
   * \return The height value of this Object.
   */
  double getHeight() const;

  /*!
   * \brief Height value mutator.
   *
   * Set the height value of this Object.
   *
   * \param height The new height value of this Object.
   */
  void setHeight(const double height);

  /*!
   * \brief Aliases value accessor.
   *
   * Get the aliases of this Object.
   *
   * \return The aliases of this Object.
   */
  const std::vector<std::string> &getAliases() const;

  /*!
   * \brief Aliases size accessor.
   *
   * Get the number of aliases of this Object.
   *
   * \return The number of aliases of this Object.
   */
  size_t getNumAliases() const;

  /*!
   * \brief Alias value accessor.
   *
   * Get the alias of this Object at the given index.
   *
   * \param i The index of the alias to get.
   * \return The alias at the given index.
   * \throws std::out_of_range Thrown if the alias at the given index does not exist.
   */
  const std::string &getAlias(const size_t index) const;

  /*!
   * \brief Alias adder.
   *
   * Add the alias to this Object.
   *
   * \param alias The new alias to add to this Object.
   */
  void addAlias(const std::string &alias);

  /*!
   * \brief Alias remover.
   *
   * Remove the alias at the given index. An invalid index results in no effect.
   *
   * \param i The index of the alias to remove.
   * \throws std::out_of_range Thrown if the alias at the given index does not exist.
   */
  void removeAlias(const size_t index);

  /*!
   * \brief Name check.
   *
   * Check if this object goes by the given name by checking both the default name and aliases. Case is not important.
   *
   * \param name The name to check,
   * \return Returns true if the name or any of the aliases of this object match the given name.
   */
  bool checkName(const std::string &name) const;

  /*!
   * \brief Position transform.
   *
   * Transform the given Position from the parent's frame to this Object's frame. For example, the parent of a
   * Surface is a Room. Passing in a Position in the Room frame will return that Position in the Surface frame.
   *
   * \param position The Position to transform,
   * \return Returns a new Position object in relation to this Object.
   */
  geometry::Position fromParentFrame(const geometry::Position &position) const;

  /*!
   * \brief Pose transform.
   *
   * Transform the given Pose from the parent's frame to this Object's frame. For example, the parent of a
   * Surface is a Room. Passing in a Pose in the Room frame will return that Pose in the Surface frame.
   *
   * \param pose The Pose to transform,
   * \return Returns a new Pose object in relation to this Object.
   */
  geometry::Pose fromParentFrame(const geometry::Pose &pose) const;

private:
  /*! Name of the object and frame ID. */
  std::string name_, frame_id_;
  /*! The pose of the object with respect to the parent's frame */
  geometry::Pose pose_;
  /*! Size of the object. */
  double width_, depth_, height_;
  /*! List of name aliases. */
  std::vector<std::string> aliases_;
};

}
}
}
}

#endif
