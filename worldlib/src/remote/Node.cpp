/*!
 * \file Node.cpp
 * \brief Abstract ROS node extension for use with common worldlib remote clients.
 *
 * A worldlib node includes common functionality that is useful for use with the worldlib such as setting up clients
 * based on global parameters.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 27, 2015
 */

// worldlib
#include "worldlib/remote/Node.h"

// ROS
#include <ros/package.h>

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::geometry;
using namespace rail::spatial_temporal_learning::worldlib::remote;

Node::Node() : private_node_("~"), tf_listener_(tfs_)
{
  okay_ = true;
}

bool Node::loadWorldYamlFile(const bool verbose)
{
  // location of the world config file
  string world_config(ros::package::getPath("worldlib") + "/config/world.yaml");
  private_node_.getParam("world_config", world_config);
  if (verbose)
  {
    ROS_INFO("World Configutation YAML: %s", world_config.c_str());
  }
  // load the config
  return world_.loadFromYaml(world_config);
}

InteractiveWorldModelClient *Node::createInteractiveWorldModelClient(const bool verbose) const
{
  // set connection defaults
  int port = InteractiveWorldModelClient::DEFAULT_PORT;
  string host("robotsfor.me");

  // grab any parameters we need
  node_.getParam("/worldlib/interactive_world_model_client/host", host);
  node_.getParam("/worldlib/interactive_world_model_client/port", port);

  // create the new client
  InteractiveWorldModelClient *client = new InteractiveWorldModelClient(host, port);

  // check verbosity
  if (verbose)
  {
    ROS_INFO("Interactive World Model Server: http://%s:%hu/", client->getHost().c_str(), client->getPort());
  }

  return client;
}

SpatialWorldClient *Node::createSpatialWorldClient(const bool verbose) const
{
  // set connection defaults
  int port = SpatialWorldClient::DEFAULT_PORT;
  string host("localhost");
  string user("ros");
  string password("");
  string database("rms");

  // grab any parameters we need
  node_.getParam("/worldlib/spatial_world_client/host", host);
  node_.getParam("/worldlib/spatial_world_client/port", port);
  node_.getParam("/worldlib/spatial_world_client/user", user);
  node_.getParam("/worldlib/spatial_world_client/password", password);
  node_.getParam("/worldlib/spatial_world_client/database", database);

  // create the new client
  SpatialWorldClient *client = new SpatialWorldClient(host, port, user, password, database);

  // check verbosity
  if (verbose)
  {
    ROS_INFO("Spatial World Server: mysql://%s@%s:%hu/%s (Using Password: %s)", client->getUser().c_str(),
             client->getHost().c_str(), client->getPort(), client->getDatabase().c_str(),
             (client->getPassword().empty()) ? "NO" : "YES");
  }

  return client;
}

bool Node::okay() const
{
  return okay_;
}

Pose Node::transformToWorld(const Pose &pose, const string &pose_frame_id) const
{
  // get the transform from the frame origin to the world frame
  const worldlib::geometry::Pose p_frame_world(tfs_.lookupTransform(world_.getFixedFrameID(), pose_frame_id,
                                                                    ros::Time(0)).transform);
  // multiply to get the pose in the world frame
  const tf2::Transform t_pose_frame = pose.toTF2Transform();
  const tf2::Transform t_frame_world = p_frame_world.toTF2Transform();
  const worldlib::geometry::Pose p_pose_world(t_frame_world * t_pose_frame);
  return p_pose_world;
}
