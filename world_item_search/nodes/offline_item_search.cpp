/*!
 * \file offline_item_search.cpp
 * \brief An offline object search example node.
 *
 * The offline item search is an example item search node. This node will fake observation data and build models from
 * the faked data. This is useful for model verification methods and data collection for ongoing research.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 27, 2015
 */

#include "world_item_search/OfflineItemSearcher.h"

using namespace std;
using namespace rail::spatial_temporal_learning;

/*!
 * Creates and runs the offline_item_search node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly or EXIT_FAILURE if an error occurs.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "offline_item_search");
  OfflineItemSearcher searcher;
  // check if everything started okay
  if (searcher.okay())
  {
    // run the process
    searcher.run();
    return EXIT_SUCCESS;
  } else
  {
    return EXIT_FAILURE;
  }
}
