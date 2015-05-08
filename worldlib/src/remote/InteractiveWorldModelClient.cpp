/*!
 * \file InteractiveWorldModelClient.cpp
 * \brief The main HTTP client connection to the interactive world trained models.
 *
 * The interactive world model client performs HTTP requests to the trained models from the interactive world.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 25, 2015
 */

// worldlib
#include "worldlib/geometry/Pose.h"
#include "worldlib/model/PlacementModel.h"
#include "worldlib/remote/InteractiveWorldModelClient.h"

// ROS
#include <ros/ros.h>

// JSON
#include <jsoncpp/json/json.h>

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::geometry;
using namespace rail::spatial_temporal_learning::worldlib::model;
using namespace rail::spatial_temporal_learning::worldlib::remote;
using namespace rail::spatial_temporal_learning::worldlib::world;

InteractiveWorldModelClient::InteractiveWorldModelClient(const InteractiveWorldModelClient &client) : HttpClient(client)
{
}

InteractiveWorldModelClient::InteractiveWorldModelClient(const string &host, const uint16_t port)
    : HttpClient(host, port)
{
}

bool InteractiveWorldModelClient::getTaskModel(const uint32_t task_id, TaskModel &task_model) const
{
  // build the URL request
  stringstream ss;
  ss << "iwmodels/view/" << task_id;

  // make the GET request
  const string &response = this->get(ss.str());
  // check the size
  if (response.empty())
  {
    ROS_WARN("No interactive world models found for task ID %lu.", (ulong) task_id);
    return false;
  } else
  {
    // set the ID
    task_model.setTaskID(task_id);

    // parse the root of the JSON response
    Json::Reader reader;
    Json::Value json_response;
    reader.parse(response, json_response, false);
    const Json::Value &iwmodel = json_response["Iwmodel"];
    const string &value = iwmodel["value"].asString();

    // parse the models array
    Json::Value value_json;
    reader.parse(value, value_json, false);
    const Json::Value &models = value_json["models"];
    // go through each
    for (int i = 0; i < models.size(); i++)
    {
      const Json::Value &json_model = models[i];

      // parse the Gaussian values
      const double decision_value = json_model["decision_value"].asDouble();
      const double sigma_x = json_model["sigma_x"].asDouble();
      const double sigma_y = json_model["sigma_y"].asDouble();
      const double sigma_theta = json_model["sigma_theta"].asDouble();

      // parse the placement
      const Json::Value &json_placement = json_model["placement"];
      const Json::Value &json_position = json_placement["position"];
      const Position placement_position(json_position["x"].asDouble(), json_position["y"].asDouble(),
                                        json_position["z"].asDouble());
      const Orientation placement_orientation(json_placement["rotation"].asDouble());
      const Pose placement_pose(placement_position, placement_orientation);

      // parse the item
      const Json::Value &json_item = json_placement["item"];
      const string &item_name = json_item["name"].asString();
      // sizes are only in two dimensions in the interactive world
      const double item_width = json_item["width"].asDouble();
      const double item_depth = json_item["height"].asDouble();
      // there is no reference frame for the item in the interactive world
      const Item item(item_name, "", placement_pose, item_width, item_depth);

      // there is no reference frame for the reference object in the interactive world (they are indexed by names)
      const Object object(json_placement["reference_frame_id"].asString());

      // parse the surface it was placed on
      const Json::Value &json_surface = json_placement["surface"];
      const string &surface_name = json_surface["name"].asString();
      // sizes are only in two dimensions in the interactive world
      const double surface_width = json_surface["width"].asDouble();
      const double surface_depth = json_surface["height"].asDouble();
      // there is no reference frame for the item in the interactive world (they are indexed by names)
      const Surface surface(surface_name, "", Pose(), surface_width, surface_depth);

      // build the final model
      Placement placement(item, object, surface);
      PlacementModel placement_model(placement, decision_value, sigma_x, sigma_y, sigma_theta);

      task_model.addPlacementModel(placement_model);
    }

    return true;
  }
}

bool InteractiveWorldModelClient::getTaskItems(const uint32_t task_id, vector<Item> &items) const
{
  // load the task model
  TaskModel task_model;
  if (this->getTaskModel(task_id, task_model))
  {
    task_model.getUniqueItems(items);
    return items.size() > 0;
  } else
  {
    return false;
  }
}

bool InteractiveWorldModelClient::getTaskSurfaces(const uint32_t task_id, vector<Surface> &surfaces) const
{
  // load the task model
  TaskModel task_model;
  if (this->getTaskModel(task_id, task_model))
  {
    task_model.getUniqueSurfaces(surfaces);
    return surfaces.size() > 0;
  } else
  {
    return false;
  }
}
