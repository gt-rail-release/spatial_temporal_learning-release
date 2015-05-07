/*!
 * \file OfflineItemSearcher.h
 * \brief An offline object search example node.
 *
 * The offline item search is an example item search node. This node will fake observation data and build models from
 * the faked data. This is useful for model verification methods and data collection for ongoing research.
 *
 * Note that this node will clear the spatial world database several times. You should only run this node on a test
 * database, not your robot's persistent database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 27, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_OFFLINE_ITEM_SEARCHER_H_
#define SPATIAL_TEMPORAL_LEARNING_OFFLINE_ITEM_SEARCHER_H_

// World Item Search
#include "GeoLifeEntry.h"

// worldlib
#include "worldlib/remote/Node.h"
#include "worldlib/model/TaskModel.h"
#include "worldlib/world/World.h"

// Boost
#include <boost/random.hpp>

// C++ Standard Library
#include <vector>

namespace rail
{
namespace spatial_temporal_learning
{

/*!
 * \class OfflineItemSearcher
 * \brief An offline object search example node.
 *
 * The offline item search is an example item search node. This node will fake observation data and build models from
 * the faked data. This is useful for model verification methods and data collection for ongoing research.
 */
class OfflineItemSearcher : public worldlib::remote::Node
{
public:
  /*! The number of rows in the 2D GeoLife grid. */
  static const int NUM_GEOLIFE_ROWS = 10;
  /*! The number of columns in the 2D GeoLife grid. */
  static const int NUM_GEOLIFE_COLUMNS = 10;

  /*!
   * \brief Create a OfflineItemSearcher and associated ROS information.
   *
   * Creates the ROS node handle and creates clients to the worldlib databases.
   */
  OfflineItemSearcher();

  /*!
   * \brief Cleans up a OfflineItemSearcher.
   *
   * Cleans up any connections used by the OfflineItemSearcher.
   */
  virtual ~OfflineItemSearcher();

  /*!
   * \brief Run the offline item search process.
   *
   * Run the simulated item search process and print out the output.
   */
  void run();

private:
  /*!
   * \brief Load GeoLife PLT log files.
   *
   * Load GeoLife trajectory files as a means of model verification testing. This will load all PLT files from the
   * given directory and place them in the searcher's vector to utilize later.
   *
   * \param directory The directory to search for PLT files in.
   */
  void loadGeoLife(const std::string &directory);

  /*!
   * \brief Run the GeoLife persistence model experiment.
   *
   * Run the GeoLife persistence model experiment. This experiment will randomly pick points from the first 2/3 of
   * the GeoLife entries and store them as observations before building a persistence model and validating that model.
   */
  void runGeoLifeExperiment();

  /*!
   * \brief Determine the "surface" name based on latitude and longitude.
   *
   * Determine the "surface" name based on the 2D grid overlayed on the map. This will be a surface such as "2-3" for
   * row two, column three.
   *
   * \param latitude The latitude of the point.
   * \param longitude The longitude of the point.
   * \return The string "surface" name.
   */
  std::string getGeoLifeSurface(const double latitude, const double longitude) const;

  /*!
   * \brief Print the list of items to standard out.
   *
   * Prints the given list of items to standard out.
   *
   * \param items The item list to print.
   */
  void printItemList(const std::vector<worldlib::world::Item> &objects) const;

  /*!
   * \brief Print the list of surfaces to standard out.
   *
   * Prints the given list of surfaces to standard out.
   *
   * \param surfaces The surface list to print.
   */
  void printSurfaceList(const std::vector<worldlib::world::Surface> &surfaces) const;

  /*! Random number generator. */
  boost::mt19937 random_;

  /*! The data entry values from the GeoLife dataset. */
  std::vector<GeoLifeEntry> geolife_;
  /*! Bounding region for the GeoLife data */
  double min_lat_, max_lat_, min_lng_, max_lng_;

  /*! The interactive world model client */
  worldlib::remote::InteractiveWorldModelClient *interactive_world_model_client_;
  /*! The spatial world database client */
  worldlib::remote::SpatialWorldClient *spatial_world_client_;

  /*! Our "worlds". */
  worldlib::world::World world_, interactive_world_;

  /*! The interactive world task model for putting items away. */
  worldlib::model::TaskModel interactive_world_task_model_;
};

}
}

#endif