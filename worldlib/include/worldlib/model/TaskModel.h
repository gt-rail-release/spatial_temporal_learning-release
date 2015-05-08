/*!
 * \file TaskModel.h
 * \brief Task placement model information.
 *
 * A task model contains a set of placement models representing the strength of reference frames for a given task.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 27, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_MODEL_TASK_MODEL_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_MODEL_TASK_MODEL_H_

// worldlib
#include "PlacementModel.h"

// C++ Standard Library
#include <stdint.h>
#include <string>
#include <vector>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace model
{

/*!
 * \class TaskModel
 * \brief Task placement model information.
 *
 * A task model contains a set of placement models representing the strength of reference frames for a given task.
 */
class TaskModel
{
public:
  /*!
   * \brief Create a new TaskModel.
   *
   * Create a new empty TaskModel with the given parameters.
   *
   * \param task_id A unique identifier for the task (defaults to 0).
   * \param name The name of the task (defaults to the empty string).
   */
  TaskModel(const uint32_t task_id = 0, const std::string &name = "");

  /*!
   * \brief Task ID value accessor.
   *
   * Get the task ID value of this TaskModel.
   *
   * \return The task ID value of this TaskModel.
   */
  uint32_t getTaskID();

  /*!
   * \brief Task ID value mutator.
   *
   * Set the task ID value of this TaskModel.
   *
   * \param task_id The new task ID value of this TaskModel.
   */
  void setTaskID(const uint32_t task_id);

  /*!
   * \brief Name value accessor.
   *
   * Get the name value of this TaskModel.
   *
   * \return The name value of this TaskModel.
   */
  const std::string &getName() const;

  /*!
   * \brief Name value mutator.
   *
   * Set the name value of this TaskModel.
   *
   * \param name The new name value of this TaskModel.
   */
  void setName(const std::string &name);

  /*!
   * \brief Placement models value accessor.
   *
   * Get the placement models of this TaskModel.
   *
   * \return The placement models.
   */
  const std::vector<PlacementModel> &getPlacementModels() const;

  /*!
   * \brief Placement models value accessor (immutable).
   *
   * Get the placement models of this TaskModel.
   *
   * \return The placement models.
   */
  std::vector<PlacementModel> &getPlacementModels();

  /*!
   * \brief Placement models size accessor.
   *
   * Get the number of placement models of this TaskModel.
   *
   * \return The number of placement models of this TaskModel.
   */
  size_t getNumPlacementModels() const;

  /*!
   * \brief PlacementModel value accessor (immutable).
   *
   * Get the PlacementModel of this TaskModel at the given index.
   *
   * \param i The index of the PlacementModel to get.
   * \return The PlacementModel at the given index.
   * \throws std::out_of_range Thrown if the PlacementModel at the given index does not exist.
   */
  const PlacementModel &getPlacementModel(const size_t index) const;

  /*!
   * \brief PlacementModel value accessor.
   *
   * Get the PlacementModel of this TaskModel at the given index.
   *
   * \param i The index of the PlacementModel to get.
   * \return The PlacementModel at the given index.
   * \throws std::out_of_range Thrown if the PlacementModel at the given index does not exist.
   */
  PlacementModel &getPlacementModel(const size_t index);

  /*!
   * \brief PlacementModel adder.
   *
   * Add the PlacementModel to this World.
   *
   * \param placement_model The new PlacementModel to add.
   */
  void addPlacementModel(const PlacementModel &placement_model);

  /*!
   * \brief PlacementModel remover.
   *
   * Remove the PlacementModel at the given index. An invalid index results in no effect.
   *
   * \param i The index of the PlacementModel pose to remove.
   * \throws std::out_of_range Thrown if the PlacementModel at the given index does not exist.
   */
  void removePlacementModel(const size_t index);

  /*!
   * \brief Get a unique list of the items in the task.
   *
   * Get a unique list of the items in the task. All poses are set to 0 since they are not placed items, simply a
   * list of the items for the task.
   *
   * \param items The vector to fill with the unique items.
   */
  void getUniqueItems(std::vector<world::Item> &items) const;

  /*!
   * \brief Get a unique list of the surfaces in the task.
   *
   * Get a unique list of the surfaces in the task. All poses are set to 0 since they are not placed surfaces, simply a
   * list of the surfaces for the task.
   *
   * \param surfaces The vector to fill with the unique surfaces.
   */
  void getUniqueSurfaces(std::vector<world::Surface> &surfaces) const;

private:
  /*! The ID of the task. */
  uint32_t task_id_;
  /*! The name of the task. */
  std::string name_;
  /* The associated placement models */
  std::vector<PlacementModel> placement_models_;
};

}
}
}
}

#endif
