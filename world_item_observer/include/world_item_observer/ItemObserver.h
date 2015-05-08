/*!
 * \file ItemObserver.h
 * \brief A persistent observer of items in the world for the spatial world database.
 *
 * The world item observer will store item observations in a remote spatial world database by listening to a
 * rail_manipulation_msgs/SegmentedObjectList message.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date May 5, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_ITEM_OBSERVER_H_
#define SPATIAL_TEMPORAL_LEARNING_ITEM_OBSERVER_H_

// worldlib
#include "worldlib/remote/Node.h"

// ROS
#include <rail_manipulation_msgs/SegmentedObjectList.h>

namespace rail
{
namespace spatial_temporal_learning
{

/*!
 * \class ItemObserver
 * \brief A persistent observer of items in the world for the spatial world database.
 *
 * The world item observer will store item observations in a remote spatial world database by listening to a
 * rail_manipulation_msgs/SegmentedObjectList message.
 */
class ItemObserver : public worldlib::remote::Node
{
public:
  /*!
   * \brief Create a ItemObserver and associated ROS information.
   *
   * Creates the ROS node handle and creates clients to the worldlib database and object topic.
   */
  ItemObserver();

  /*!
   * \brief Cleans up a ItemObserver.
   *
   * Cleans up any connections used by the ItemObserver.
   */
  virtual ~ItemObserver();

private:
  /*!
   * \brief Main callback for the recognized objects topic.
   *
   * The recognized objects callback will add all observations seen based on the recognized objects and the surface
   * from the world they are most likely on (based on bounding regions). Any missing items will be marked as removed.
   *
   * \param objects The current list of segmented objects.
   */
  void recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectListConstPtr &objects) const;

  /*! The spatial world database client */
  worldlib::remote::SpatialWorldClient *spatial_world_client_;

  /*! The base frame of the robot. */
  std::string base_frame_id_;

  /*! The recognized objects topic. */
  ros::Subscriber recognized_objects_sub_;
};

}
}

#endif