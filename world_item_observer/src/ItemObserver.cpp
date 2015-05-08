/*!
 * \file ItemObserver.cpp
 * \brief A persistent observer of items in the world for the spatial world database.
 *
 * The world item observer will store item observations in a remote spatial world database by listening to a
 * rail_manipulation_msgs/SegmentedObjectList message.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date May 5, 2015
 */

// World Item Observer
#include "world_item_observer/ItemObserver.h"

// ROS
#include <graspdb/Client.h>

using namespace std;
using namespace rail::spatial_temporal_learning;
using namespace rail::pick_and_place;

ItemObserver::ItemObserver() : worldlib::remote::Node(), base_frame_id_("base_footprint")
{
  // check the base frame ID
  private_node_.getParam("base_frame_id", base_frame_id_);

  // load the config
  okay_ &= this->loadWorldYamlFile();

  // create the client we need
  spatial_world_client_ = this->createSpatialWorldClient();
  okay_ &= spatial_world_client_->connect();

  // connect to the grasp model database to check for items
  int port = pick_and_place::graspdb::Client::DEFAULT_PORT;
  string host("127.0.0.1");
  string user("ros");
  string password("");
  string db("graspdb");
  node_.getParam("/graspdb/host", host);
  node_.getParam("/graspdb/port", port);
  node_.getParam("/graspdb/user", user);
  node_.getParam("/graspdb/password", password);
  node_.getParam("/graspdb/db", db);
  pick_and_place::graspdb::Client client(host, port, user, password, db);

  if (client.connect())
  {
    // add the items
    vector<string> objects;
    client.getUniqueGraspModelObjectNames(objects);
    for (size_t i = 0; i < objects.size(); i++)
    {
      world_.addItem(worldlib::world::Item(objects[i]));
    }
    ROS_INFO("Found %lu unique items in the world.", world_.getNumItems());
    okay_ &= true;
  }

  // connect to the segmented objects topic
  string recognized_objects_topic("/object_recognition_listener/recognized_objects");
  private_node_.getParam("recognized_objects_topic", recognized_objects_topic);
  recognized_objects_sub_ = node_.subscribe(recognized_objects_topic, 1, &ItemObserver::recognizedObjectsCallback,
                                            this);

  if (okay_)
  {
    ROS_INFO("Item Observer Initialized");
  }
}

ItemObserver::~ItemObserver()
{
  // clean up the client
  delete spatial_world_client_;
}

void ItemObserver::recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectListConstPtr &objects) const
{
  // only work on a non-cleared list
  if (!objects->cleared)
  {
    size_t new_observation_count = 0;
    // used to keep track of what was seen where (to mark things as missing)
    map<string, vector<string> > seen;
    // only utilize recognized objects
    for (size_t i = 0; i < objects->objects.size(); i++)
    {
      const rail_manipulation_msgs::SegmentedObject &o = objects->objects[i];
      // transform the centroid to the fixed frame (shift down the Z)
      const worldlib::geometry::Position offset(o.centroid.x, o.centroid.y, o.centroid.z - (o.height / 2.0));
      const worldlib::geometry::Pose centroid(offset, worldlib::geometry::Orientation(o.orientation));
      const string &frame_id = o.point_cloud.header.frame_id;
      const worldlib::geometry::Pose p_centroid_world = this->transformToWorld(centroid, frame_id);

      // check if it is on a surface
      size_t room_i, surface_i, placement_surface_i;
      if (world_.findPlacementSurface(p_centroid_world.getPosition(), room_i, surface_i, placement_surface_i))
      {
        const worldlib::world::Room &room = world_.getRoom(room_i);
        const worldlib::world::Surface &surface = room.getSurface(surface_i);
        // check if it is recognized, otherwise just mark the surface as being seen
        if (o.recognized)
        {
          // determine the position of the item on the surface
          const worldlib::geometry::Pose p_centroid_room = room.fromParentFrame(p_centroid_world);
          const worldlib::geometry::Pose p_centroid_surface = surface.fromParentFrame(p_centroid_room);

          // create and store the Observation
          const worldlib::world::Item item(o.name, "", p_centroid_surface, o.width, o.depth, o.height);
          spatial_world_client_->addObservation(item, surface, p_centroid_surface);
          new_observation_count++;
          seen[surface.getName()].push_back(item.getName());
        } else
        {
          seen[surface.getName()].push_back("");
        }
      }
    }

    // special case -- if nothing was seen, we should mark the closest surface as being empty
    if (seen.empty())
    {
      const worldlib::geometry::Pose p_robot_world = this->transformToWorld(worldlib::geometry::Pose(), base_frame_id_);
      size_t room_i, surface_i;
      world_.findClosestSurface(p_robot_world.getPosition(), room_i, surface_i);
      seen[world_.getRoom(room_i).getSurface(surface_i).getName()].push_back("");
    }

    // check what items were no longer seen on the surface
    size_t removed_count = 0;
    for (map<string, vector<string> >::iterator iter = seen.begin(); iter != seen.end(); ++iter)
    {
      const string &surface_name = iter->first;
      const vector<string> &items_seen = iter->second;
      const vector<worldlib::world::Item> &world_items = world_.getItems();
      for (size_t i = 0; i < world_items.size(); i++)
      {
        // check if the item was seen
        const worldlib::world::Item &cur = world_items[i];
        if (std::find(items_seen.begin(), items_seen.end(), cur.getName()) == items_seen.end())
        {
          // check if the item had been there previously
          if (spatial_world_client_->itemExistsOnSurface(cur.getName(), surface_name))
          {
            // mark it as removed
            spatial_world_client_->markObservationsAsRemoved(cur.getName(), surface_name);
            removed_count++;
          }
        }
      }
    }

    ROS_INFO("Added %lu new observations and marked %lu as removed.", new_observation_count, removed_count);
  }
}
