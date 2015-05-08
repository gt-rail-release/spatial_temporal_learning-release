/*!
 * \file world_item_observer.cpp
 * \brief A persistent observer of items in the world for the spatial world database.
 *
 * The world item observer will store item observations in a remote spatial world database by listening to a
 * rail_manipulation_msgs/SegmentedObjectList message.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date May 5, 2015
 */

#include "world_item_observer/ItemObserver.h"

using namespace std;
using namespace rail::spatial_temporal_learning;

/*!
 * Creates and runs the world_item_observer node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly or EXIT_FAILURE if an error occurs.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "world_item_observer");
  ItemObserver observer;
  // check if everything started okay
  if (observer.okay())
  {
    ros::spin();
    return EXIT_SUCCESS;
  } else
  {
    return EXIT_FAILURE;
  }
}
