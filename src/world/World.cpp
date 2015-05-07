/*!
 * \file World.cpp
 * \brief World configuration information.
 *
 * A world consists of a series of rooms, surfaces, and items. Surfaces can have points of interest as well.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

// worldlib
#include "worldlib/geometry/Pose.h"
#include "worldlib/world/World.h"

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::geometry;
using namespace rail::spatial_temporal_learning::worldlib::world;

World::World(const string &fixed_frame_id) : fixed_frame_id_(fixed_frame_id)
{
}

bool World::loadFromYaml(const std::string &file)
{
// check the YAML version
#ifdef YAMLCPP_GT_0_5_0
  try
  {
    // TF buffer
    tf2_ros::Buffer tfs;
    const tf2_ros::TransformListener tf_listener(tfs);
    // give the buffer time to fill (half second)
    usleep(500000);

    // load the config file
    const YAML::Node world = YAML::LoadFile(file);
    // set the fixed frame
    fixed_frame_id_ = world["fixed_frame_id"].as<string>();

    // parse the rooms
    const YAML::Node &rooms = world["rooms"];
    for (size_t i = 0; i < rooms.size(); i++)
    {
      const YAML::Node &room = rooms[i];
      // get the name, frame ID, dimensions, and aliases
      const Object room_object = this->parseObject(room);
      // transform to get the Pose
      const geometry_msgs::TransformStamped room_tf = tfs.lookupTransform(fixed_frame_id_, room_object.getFrameID(),
                                                                          ros::Time(0));
      const Pose room_pose(room_tf.transform);
      // add the room
      rooms_.push_back(Room(room_object.getName(), room_object.getFrameID(), room_pose, room_object.getWidth(),
                            room_object.getDepth(), room_object.getHeight()));
      Room &cur_room = rooms_.back();

      // parse the surfaces
      const YAML::Node &surfaces = room["surfaces"];
      for (size_t j = 0; j < surfaces.size(); j++)
      {
        const YAML::Node &surface = surfaces[j];
        // get the name, frame ID, dimensions, and aliases
        const Object surface_object = this->parseObject(surface);
        // transform to get the Pose
        const geometry_msgs::TransformStamped surface_tf = tfs.lookupTransform(room_object.getFrameID(),
                                                                               surface_object.getFrameID(),
                                                                               ros::Time(0));
        const Pose surface_pose(surface_tf.transform);
        // add the surface
        cur_room.addSurface(Surface(surface_object.getName(), surface_object.getFrameID(), surface_pose,
                                    surface_object.getWidth(), surface_object.getDepth(),
                                    surface_object.getHeight()));
        Surface &cur_surface = cur_room.getSurfaces().back();

        // parse the placement surfaces
        const YAML::Node &placement_surfaces = surface["placement_surfaces"];
        for (size_t k = 0; k < placement_surfaces.size(); k++)
        {
          const YAML::Node &ps = placement_surfaces[k];
          // get the name, frame ID, dimensions, and aliases
          const Object ps_object = this->parseObject(ps);
          // parse the nav frame ID
          const string ps_nav_frame_id = ps["nav_frame_id"].as<string>();
          // transform to get the Pose
          const geometry_msgs::TransformStamped ps_tf = tfs.lookupTransform(surface_object.getFrameID(),
                                                                            ps_object.getFrameID(),
                                                                            ros::Time(0));
          const Pose ps_pose(ps_tf.transform);
          // add the placement surface
          cur_surface.addPlacementSurface(PlacementSurface(ps_object.getName(), ps_object.getFrameID(), ps_nav_frame_id,
                                                           ps_pose, ps_object.getWidth(), ps_object.getDepth(),
                                                           ps_object.getHeight()));
        }
        // parse the points of interest
        const YAML::Node &pois = surface["pois"];
        for (size_t k = 0; k < pois.size(); k++)
        {
          const YAML::Node &poi = pois[k];
          // get the name, frame ID, dimensions, and aliases
          const Object poi_object = this->parseObject(poi);
          // transform to get the Pose
          const geometry_msgs::TransformStamped poi_tf = tfs.lookupTransform(surface_object.getFrameID(),
                                                                             poi_object.getFrameID(),
                                                                             ros::Time(0));
          const Pose poi_pose(poi_tf.transform);
          // add the point of interest
          cur_surface.addPointOfInterest(PointOfInterest(poi_object.getName(), poi_object.getFrameID(), poi_pose,
                                                         poi_object.getWidth(), poi_object.getDepth(),
                                                         poi_object.getHeight()));
        }
      }
    }
    return true;
  } catch (YAML::Exception &e)
  {
    ROS_ERROR("World Configuration YAML Parser Error: %s", e.msg.c_str());
    return false;
  } catch (tf2::LookupException &e)
  {
    ROS_ERROR("World Configuration TF Error: %s", e.what());
    return false;
  }
#else
  ROS_ERROR("Unsupported version of YAML. World configuration file could not be parsed.");
  return false;
#endif
}

#ifdef YAMLCPP_GT_0_5_0
Object World::parseObject(const YAML::Node &object) const
{
  // get the name, frame ID, and dimensions
  const string name = object["name"].as<string>();
  const string frame_id = object["frame_id"].as<string>();
  const double width = object["width"].IsDefined() ? object["width"].as<double>() : 0.0;
  const double depth = object["depth"].IsDefined() ? object["depth"].as<double>() : 0.0;
  const double height = object["height"].IsDefined() ? object["height"].as<double>() : 0.0;

  // create the object
  Object o(name, frame_id, Pose(), width, depth, height);

  // check for aliases
  const YAML::Node &aliases = object["aliases"];
  if (aliases.IsDefined())
  {
    for (size_t i = 0; i < aliases.size(); i++)
    {
      o.addAlias(aliases[i].as<string>());
    }
  }

  return o;
}
#endif

const string &World::getFixedFrameID() const
{
  return fixed_frame_id_;
}

void World::setFixedFrameID(const string &fixed_frame_id)
{
  fixed_frame_id_ = fixed_frame_id;
}

const vector<Room> &World::getRooms() const
{
  return rooms_;
}

vector<Room> &World::getRooms()
{
  return rooms_;
}

size_t World::getNumRooms() const
{
  return rooms_.size();
}

const Room &World::getRoom(const size_t index) const
{
  // check the index value first
  if (index < rooms_.size())
  {
    return rooms_[index];
  } else
  {
    throw out_of_range("World::getRoom : Room index does not exist.");
  }
}

Room &World::getRoom(const size_t index)
{
  // check the index value first
  if (index < rooms_.size())
  {
    return rooms_[index];
  } else
  {
    throw out_of_range("World::getRoom : Room index does not exist.");
  }
}

void World::addRoom(const Room &room)
{
  rooms_.push_back(room);
}

void World::removeRoom(const size_t index)
{
  // check the index value first
  if (index < rooms_.size())
  {
    rooms_.erase(rooms_.begin() + index);
  } else
  {
    throw out_of_range("World::removeRoom : Room index does not exist.");
  }
}

bool World::roomExists(const string &name) const
{
  // check if we can find it and catch any exceptions
  try
  {
    this->findRoom(name);
    return true;
  } catch (out_of_range &e)
  {
    return false;
  }
}

const Room &World::findRoom(const string &name) const
{
  // check each surface
  for (size_t i = 0; i < rooms_.size(); i++)
  {
    // perform a check
    if (rooms_[i].checkName(name))
    {
      return rooms_[i];
    }
  }
  // no match found
  throw out_of_range("World::findRoom : Room name does not exist.");
}


Room &World::findRoom(const string &name)
{
  // check each surface
  for (size_t i = 0; i < rooms_.size(); i++)
  {
    // perform a check
    if (rooms_[i].checkName(name))
    {
      return rooms_[i];
    }
  }
  // no match found
  throw out_of_range("World::findRoom : Room name does not exist.");
}

const vector<Item> &World::getItems() const
{
  return items_;
}

vector<Item> &World::getItems()
{
  return items_;
}

size_t World::getNumItems() const
{
  return items_.size();
}

const Item &World::getItem(const size_t index) const
{
  // check the index value first
  if (index < items_.size())
  {
    return items_[index];
  } else
  {
    throw out_of_range("World::getItem : Item index does not exist.");
  }
}

Item &World::getItem(const size_t index)
{
  // check the index value first
  if (index < items_.size())
  {
    return items_[index];
  } else
  {
    throw out_of_range("World::getItem : Item index does not exist.");
  }
}

void World::addItem(const Item &item)
{
  items_.push_back(item);
}

void World::removeItem(const size_t index)
{
  // check the index value first
  if (index < items_.size())
  {
    items_.erase(items_.begin() + index);
  } else
  {
    throw out_of_range("World::removeItem : Item index does not exist.");
  }
}

bool World::itemExists(const string &name) const
{
  // check if we can find it and catch any exceptions
  try
  {
    this->findItem(name);
    return true;
  } catch (out_of_range &e)
  {
    return false;
  }
}

const Item &World::findItem(const string &name) const
{
  // check each surface
  for (size_t i = 0; i < items_.size(); i++)
  {
    // perform a check
    if (items_[i].checkName(name))
    {
      return items_[i];
    }
  }
  // no match found
  throw out_of_range("World::findItem : Item name does not exist.");
}

Item &World::findItem(const string &name)
{
  // check each surface
  for (size_t i = 0; i < items_.size(); i++)
  {
    // perform a check
    if (items_[i].checkName(name))
    {
      return items_[i];
    }
  }
  // no match found
  throw out_of_range("World::findItem : Item name does not exist.");
}

void World::findClosestSurface(const Position &position, size_t &room_index, size_t &surface_index) const
{
  bool found = false;
  // check each room
  double best = numeric_limits<double>::infinity();
  for (size_t i = 0; i < rooms_.size(); i++)
  {
    const Room &room = rooms_[i];
    const tf2::Transform t_room_world = room.getPose().toTF2Transform();
    if (room.getNumSurfaces() > 0)
    {
      // check for the closest surface
      for (size_t j = 0; j < room.getNumSurfaces(); j++)
      {
        // get the surface in the world frame
        const Surface &surface = room.getSurface(j);
        const tf2::Transform t_surface_room = surface.getPose().toTF2Transform();
        const Pose p_surface_world(t_room_world * t_surface_room);
        double distance = p_surface_world.getPosition().distance(position);
        if (distance < best)
        {
          best = distance;
          room_index = i;
          surface_index = j;
          found = true;
        }
      }
    }
  }

  // check if a valid match was found
  if (!found)
  {
    throw out_of_range("World::findClosestSurface : No surfaces exist.");
  }
}

bool World::findPlacementSurface(const Position &position, size_t &room_index, size_t &surface_index,
    size_t &placement_surface_index) const
{
  const tf2::Transform t_pose_world = Pose(position).toTF2Transform();
  // check each room
  for (size_t i = 0; i < rooms_.size(); i++)
  {
    const Room &room = rooms_[i];
    const tf2::Transform t_room_world = room.getPose().toTF2Transform();
    if (room.getNumSurfaces() > 0)
    {
      // check each surface
      for (size_t j = 0; j < room.getNumSurfaces(); j++)
      {
        // get the surface in the world frame
        const Surface &surface = room.getSurface(j);
        const tf2::Transform t_surface_room = surface.getPose().toTF2Transform();
        const tf2::Transform t_surface_world = t_room_world * t_surface_room;

        // get the pose in relation to the surface
        const Pose p_pose_surface(t_surface_world.inverseTimes(t_pose_world));

        // first check if we are inside that surface's bounding box
        if ((surface.getWidth() / 2.0 >= abs(p_pose_surface.getPosition().getX()))
            && (surface.getDepth() / 2.0 >= abs(p_pose_surface.getPosition().getY())))
        {
          try
          {
            room_index = i;
            surface_index = j;
            placement_surface_index = surface.findClosestPlacementSurfaceIndex(p_pose_surface.getPosition());
            return true;
          } catch (std::out_of_range &e)
          {
            // means no PlacementSurface found, keep going
          }
        }
      }
    }
  }

  // nothing found
  return false;
}