/*!
 * \file Node.h
 * \brief Abstract ROS node extension for use with common worldlib remote clients.
 *
 * A worldlib node includes common functionality that is useful for use with the worldlib such as setting up clients
 * based on global parameters.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 27, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_NODE_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_NODE_H_

// worldlib
#include "InteractiveWorldModelClient.h"
#include "SpatialWorldClient.h"
#include "../geometry/Pose.h"
#include "../world/World.h"

// ROS
#include <ros/node_handle.h>
#include <tf2_ros/transform_listener.h>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace remote
{

/*!
 * \brief Abstract ROS node extension for use with common worldlib clients.
 *
 * A worldlib node includes common functionality that is useful for use with the worldlib such as setting up clients
 * based on global parameters.
 */
class Node
{
public:
  /*!
   * \brief Create a new Node.
   *
   * Create a new Node which creates both a public and private ROS node handle.
   */
  Node();

  /*!
   * \brief A check for a valid Node.
   *
   * This function will return true if the appropriate connections were created successfully during initialization.
   *
   * \return True if the appropriate connections were created successfully during initialization.
   */
  bool okay() const;

protected:
  /*!
   * \brief Load world configuration data from a YAML file.
   *
   * Loads world configuration data from a YML file by pulling parameters from the ROS parameter server.
   *
   * \param verbose If parameter information should be printed to ROS_INFO.
   * \return If the load was successful.
   */
  bool loadWorldYamlFile(const bool verbose = true);

  /*!
   * \brief Create a new InteractiveWorldModelClient.
   *
   * Create a new InteractiveWorldModelClient by pulling connection parameters from the ROS parameter server.
   *
   * \param verbose If client connection information should be printed to ROS_INFO.
   * \return A pointer to a new InteractiveWorldModelClient.
   */
  InteractiveWorldModelClient *createInteractiveWorldModelClient(const bool verbose = true) const;

  /*!
   * \brief Create a new SpatialWorldClient.
   *
   * Create a new SpatialWorldClient by pulling connection parameters from the ROS parameter server.
   *
   * \param verbose If client connection information should be printed to ROS_INFO.
   * \return A pointer to a new SpatialWorldClient.
   */
  SpatialWorldClient *createSpatialWorldClient(const bool verbose = true) const;

  /*!
   * \brief Transform the given Pose into the World frame.
   *
   * Transform the given Pose into the World frame based on TF data from the TF buffer.
   *
   * \param pose The Pose to transform.
   * \param pose_frame_id The frame ID that the Pose is in.
   * \return The Pose in respect to the World fixed frame.
   */
  geometry::Pose transformToWorld(const geometry::Pose &pose, const std::string &pose_frame_id) const;

  /*! The okay check flag. */
  bool okay_;
  /*! The world configuration. */
  worldlib::world::World world_;
  /*! The public and private ROS node handles. */
  ros::NodeHandle node_, private_node_;
  /*! TF lookup buffer. */
  tf2_ros::Buffer tfs_;
  /*! TF client. */
  tf2_ros::TransformListener tf_listener_;
};

}
}
}
}

#endif
