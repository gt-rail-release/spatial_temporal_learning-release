/*!
 * \file InteractiveWorldModelClient.h
 * \brief The main HTTP client connection to the interactive world trained models.
 *
 * The interactive world model client performs HTTP requests to the trained models from the interactive world.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 25, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_INTERACTIVE_WORLD_MODEL_CLIENT_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_INTERACTIVE_WORLD_MODEL_CLIENT_H_

// worldlib
#include "HttpClient.h"
#include "../model/TaskModel.h"
#include "../world/Item.h"

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace remote
{

/*!
 * \class InteractiveWorldModelClient
 * \brief The main HTTP client connection to the interactive world trained models.
 *
 * The interactive world model client performs HTTP requests to the trained models from the interactive world.
 */
class InteractiveWorldModelClient : public HttpClient
{
public:
  /*! The task model ID for the table setting task on the RobotsFor.Me server. */
  static const uint32_t TASK_ID_TABLE_SETTING = 1;
  /*! The task model ID for the magazine task on the RobotsFor.Me server. */
  static const uint32_t TASK_ID_MAGAZINE_PLACEMENT = 2;
  /*! The task model ID for the dirty dishes task on the RobotsFor.Me server. */
  static const uint32_t TASK_ID_DIRTY_DISHES = 3;
  /*! The task model ID for the general put away task on the RobotsFor.Me server. */
  static const uint32_t TASK_ID_PUT_AWAY_GENERAL = 4;

  /*!
   * \brief Create a new InteractiveWorldModelClient.
   *
   * Creates a new InteractiveWorldModelClient by copying the values from the given InteractiveWorldModelClient.
   *
   * \param client The InteractiveWorldModelClient to copy.
   */
  InteractiveWorldModelClient(const InteractiveWorldModelClient &client);

  /*!
   * \brief Create a new InteractiveWorldModelClient.
   *
   * Creates a new InteractiveWorldModelClient with the given connection information.
   *
   * \param host The host of the server (defaults to robotsfor.me).
   * \param port The host port of the database (defaults to 80).
   */
  InteractiveWorldModelClient(const std::string &host = "robotsfor.me", const uint16_t port = HttpClient::DEFAULT_PORT);

  /*!
   * \brief Load a task model from the remote server.
   *
   * Attempts to load a task model from the interactive world server with the given task ID and place the information
   * in the given task model.
   *
   * \param task_id The task ID to load.
   * \param task_model The TaskModel to fill.
   * \return True if no errors occurred while loading the model.
   */
  bool getTaskModel(const uint32_t task_id, model::TaskModel &task_model) const;

  /*!
   * \brief Load the list of items for a given task from the remote server.
   *
   * Attempts to load the list of items for a given task from the interactive world server with the given task ID and
   * store the information in the given vector.
   *
   * \param task_id The task ID to load the item list for.
   * \param items The vector to fill with the unique items.
   * \return True if no errors occurred while loading the model.
   */
  bool getTaskItems(const uint32_t task_id, std::vector<world::Item> &items) const;

  /*!
   * \brief Load the list of surfaces for a given task from the remote server.
   *
   * Attempts to load the list of surfaces for a given task from the interactive world server with the given task ID and
   * store the information in the given vector.
   *
   * \param task_id The task ID to load the surface list for.
   * \param surfaces The vector to fill with the unique surfaces.
   * \return True if no errors occurred while loading the model.
   */
  bool getTaskSurfaces(const uint32_t task_id, std::vector<world::Surface> &surfaces) const;
};

}
}
}
}

#endif
