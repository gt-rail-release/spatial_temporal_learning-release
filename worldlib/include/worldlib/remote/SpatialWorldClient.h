/*!
 * \file SpatialWorldClient.h
 * \brief The main MySQL client connection to the spatial world database.
 *
 * The spatial world SQL client can communicate with a MySQL database containing the spatial world database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_SPATIAL_WORLD_CLIENT_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_SPATIAL_WORLD_CLIENT_H_

// worldlib
#include "SpatialWorldObservation.h"
#include "SqlClient.h"
#include "../model/PersistenceModel.h"
#include "../world/Observation.h"

// Boost
#include <boost/random.hpp>

// C++ Standard Library
#include <vector>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace remote
{

/*!
 * \class SpatialWorldClient
 * \brief The main MySQL client connection to the spatial world database.
 *
 * The spatial world SQL client can communicate with a MySQL database containing the spatial world database.
 */
class SpatialWorldClient : public SqlClient
{
public:
  /*!
   * \brief Create a new SpatialWorldClient.
   *
   * Creates a new Client by copying the values from the given SpatialWorldClient. A new connection is made if one
   * exists.
   *
   * \param client The SpatialWorldClient to copy.
   */
  SpatialWorldClient(const SpatialWorldClient &client);

  /*!
   * \brief Create a new SpatialWorldClient.
   *
   * Creates a new SpatialWorldClient with the given connection information. A connection is not made by default.
   *
   * \param host The host of the database.
   * \param port The host port of the database.
   * \param user The user of the database.
   * \param password The password for the user of the database.
   * \param database The database name.
   */
  SpatialWorldClient(const std::string &host, const uint16_t port, const std::string &user, const std::string &password,
      const std::string &database);

  /*!
   * \brief Create a connection to the spatial world database.
   *
   * Attempts to create a connection to the spatial world database. Tables are created if they do not exist yet. A flag
   * is returned to indicate the success.
   *
   * \return True if a connection has been successfully made.
   */
  virtual bool connect();

  /*!
   * \brief Clear all entries in the spatial world database.
   *
   * Attempts to clear all entries in the spatial world database. If no connection is made, no effect is made.
   */
  void clearAllEntities() const;

  /*!
   * \brief Add several observations to the spatial world database.
   *
   * Attempts to add observations to the spatial world database. If no connection is made, no effect is made.
   *
   * \param item The Item observed in the world.
   * \param surface The Surface the Item was observed on.
   * \param pose The Pose of the Item with respect to the Surface.
   */
  void addObservations(const std::vector<world::Observation> &observations) const;

  /*!
   * \brief Add a new Observation to the spatial world database.
   *
   * Attempts to add an Observation to the spatial world database. If no connection is made, no effect is made.
   * This will create default timestamps for all values.
   *
   * \param item The Item observed in the world.
   * \param surface The Surface the Item was observed on.
   * \param pose The Pose of the Item with respect to the Surface.
   */
  void addObservation(const world::Item &item, const world::Surface &surface, const geometry::Pose &pose) const;

  /*!
   * \brief Add a new Observation to the spatial world database.
   *
   * Attempts to add an Observation to the spatial world database. If no connection is made, no effect is made.
   *
   * \param observation The Observation observed in the world.
   */
  void addObservation(const world::Observation &observation) const;

  /*!
   * \brief Get all observations for a given Item name.
   *
   * Load all observations for a given Item name and store them in the given vector. If no connection is made, no effect
   * is made.
   *
   * \param item_name The Item name to load observations for (case is not important).
   * \param observations The array of observations to fill once loaded.
   */
  void getObservationsByItemName(const std::string &item_name,
      std::vector<SpatialWorldObservation> &observations) const;

  /*!
   * \brief Get all observations for a given Surface name.
   *
   * Load all observations for a given Surface name and store them in the given vector. If no connection is made, no
   * effect is made.
   *
   * \param surface_name The Surface name to load observations for (case is not important).
   * \param observations The array of observations to fill once loaded.
   */
  void getObservationsBySurfaceName(const std::string &surface_name,
      std::vector<SpatialWorldObservation> &observations) const;

  /*!
   * \brief Get all observations for a given Item and Surface name.
   *
   * Load all observations for a given Item and Surface name and store them in the given vector. If no connection is
   * made, no effect is made.
   *
   * \param item_name The Item name to load observations for (case is not important).
   * \param surface_name The Surface name to load observations for (case is not important).
   * \param observations The array of observations to fill once loaded.
   */
  void getObservationsByItemAndSurfaceName(const std::string &item_name, const std::string &surface_name,
      std::vector<SpatialWorldObservation> &observations) const;

  /*!
   * \brief Get all observations for a given Surface frame ID.
   *
   * Load all observations for a given Surface frame ID and store them in the given vector. Note that this function
   * is case sensitive. If no connection is made, no effect is made.
   *
   * \param surface_frame_id The Surface name to load observations for (case **is** important).
   * \param observations The array of observations to fill once loaded.
   */
  void getObservationsBySurfaceFrameID(const std::string &surface_frame_id,
      std::vector<SpatialWorldObservation> &observations) const;

  /*!
   * \brief Get the most frequent surface name for the given Item.
   *
   * Get the name of the surface the the given item has been seen on the most.
   *
   * \param item_name The Item name to get the most frequent surface for (case is not important).
   * \return The name of the Surface.
   */
  std::string getMostFrequentSurfaceName(const std::string &item_name) const;

  /*!
   * \brief Get the most frequent surface name for the given Item.
   *
   * Get the name of the surface the the given item has been seen on the most.
   *
   * \param item_name The Item to get the most frequent surface for.
   * \return The name of the Surface.
   */
  std::string getMostFrequentSurfaceName(const world::Item &item) const;

  /*!
   * \brief Update the given observation.
   *
   * Update the given observation in the database with its current fields. If no connection is made, no effect is made.
   *
   * \param observation The SpatialWorldObservation to update in the database.
   */
  void updateObservation(const SpatialWorldObservation &observation) const;

  /*!
   * \brief Check if an Item still exists on a given Surface.
   *
   * Check if the Item still exists on a given Surface (i.e., it was not removed yet). If no connection is made false
   * is returned.
   *
   * \param item_name The name of the Item (case is not important).
   * \param item_name The name of the Surface (case is not important).
   * \return Returns true if the Item still exists on a given Surface (i.e., it was not removed yet).
   */
  bool itemExistsOnSurface(const std::string &item_name, const std::string &surface_name) const;

  /*!
   * \brief Check if an Item has ever been seen on a given Surface.
   *
   * Check if an Item has ever been seen on a given Surface. If no connection is made false is returned.
   *
   * \param item The Item.
   * \param surface The Surface.
   * \return Returns true if the Item has ever been observed on a given Surface.
   */
  bool itemObservedOnSurface(const world::Item &item, const world::Surface &surface) const;

  /*!
   * \brief Check if an Item has ever been seen on a given Surface.
   *
   * Check if an Item has ever been seen on a given Surface. If no connection is made false is returned.
   *
   * \param item_name The name of the Item (case is not important).
   * \param surface_name The name of the Surface (case is not important).
   * \return Returns true if the Item has ever been observed on a given Surface.
   */
  bool itemObservedOnSurface(const std::string &item_name, const std::string &surface_name) const;

  /*!
   * \brief Mark the given Item on the given Surface as being removed.
   *
   * Mark the given Item on the given Surface as being removed. This will set the observed removal time for all
   * current observations to now and set an estimated time for removal for the longest observation. If no connection is
   * made, or if the Item was not currently on the Surface, no effect is made.
   *
   * \param Item The Item.
   * \param surface The Surface.
   * \param removed_observed The time the Item was observed removed (defaults to now).
   */
  void markObservationsAsRemoved(const world::Item &item, const world::Surface &surface,
      const ros::Time &removed_observed = ros::Time::now());

  /*!
   * \brief Mark the given Item on the given Surface as being removed.
   *
   * Mark the given Item on the given Surface as being removed. This will set the observed removal time for all
   * current observations to now and set an estimated time for removal for the longest observation. If no connection is
   * made, or if the Item was not currently on the Surface, no effect is made.
   *
   * \param item_name The name of the Item (case is not important).
   * \param surface_name The name of the Surface (case is not important).
   * \param removed_observed The time the Item was observed removed (defaults to now).
   */
  void markObservationsAsRemoved(const std::string &item_name, const std::string &surface_name,
      const ros::Time &removed_observed = ros::Time::now());

  /*!
   * \brief Get a list of unique surface names.
   *
   * Get a unique list of surface names and store them in the given vector.
   *
   * \param names The vector to store the name list in.
   */
  void getUniqueSurfaceNames(std::vector<std::string> &names) const;

  /*!
   * \brief Get the PersistenceModel associated with the Item on a Surface.
   *
   * Get the PersistenceModel associated with the Item on a Surface. This will calculate parameters based on data
   * from the spatial world database.
   *
   * \param item_name The name of the Item (case is not important).
   * \param surface_name The name of the Surface (case is not important).
   * \return The PersistenceModel loaded.
   */
  model::PersistenceModel getPersistenceModel(const std::string &item_name, const std::string &surface_name) const;

  /*!
   * \brief Get the PersistenceModel associated with the Item on a Surface.
   *
   * Get the PersistenceModel associated with the Item on a Surface. This will calculate parameters based on data
   * from the spatial world database.
   *
   * \param item The Item.
   * \param surface The Surface.
   * \return The PersistenceModel loaded.
   */
  model::PersistenceModel getPersistenceModel(const world::Item &item, const world::Surface &surface) const;

private:
  /*! Random number generator. */
  boost::mt19937 random_;

  /*!
   * \brief Create the spatial world table (sws).
   *
   * Attempts to create the spatial world table (sws). If no connection is made, no effect is made.
   */
  void createTable();

  /*!
   * \brief Get all observations based on a given SQL where clause.
   *
   * Load all observations for a given where clause and store them in the given vector. If no connection is made, no
   * effect is made. This is meant for internal use only.
   *
   * \param observations The array of observations to fill once loaded.
   * \param where_clause The SQL where clause of a select on the observations.
   * \param limit The SQL return limit (defaults to 0 meaning unlimited).
   * \param order_by The SQL ordering constraint (defaults to ascending `time` field; i.e., oldest first).
   */
  void getObservationsHelper(std::vector<SpatialWorldObservation> &observations, const std::string &where_clause,
      const uint32_t limit = 0, const std::string &order_by = "`time` ASC") const;
};

}
}
}
}

#endif
