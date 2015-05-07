/*!
 * \file TaskModel.cpp
 * \brief Task placement model information.
 *
 * A task model contains a set of placement models representing the strength of reference frames for a given task.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 27, 2015
 */

// worldlib
#include "worldlib/geometry/Pose.h"
#include "worldlib/model/TaskModel.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::geometry;
using namespace rail::spatial_temporal_learning::worldlib::model;
using namespace rail::spatial_temporal_learning::worldlib::world;

TaskModel::TaskModel(const uint32_t task_id, const string &name) : name_(name)
{
  task_id_ = task_id;
}

uint32_t TaskModel::getTaskID()
{
  return task_id_;
}

void TaskModel::setTaskID(const uint32_t task_id)
{
  task_id_ = task_id;
}

const string &TaskModel::getName() const
{
  return name_;
}

void TaskModel::setName(const std::string &name)
{
  name_ = name;
}

const vector<PlacementModel> &TaskModel::getPlacementModels() const
{
  return placement_models_;
}

vector<PlacementModel> &TaskModel::getPlacementModels()
{
  return placement_models_;
}

size_t TaskModel::getNumPlacementModels() const
{
  return placement_models_.size();
}

const PlacementModel &TaskModel::getPlacementModel(const size_t index) const
{
  // check the index value first
  if (index < placement_models_.size())
  {
    return placement_models_[index];
  } else
  {
    throw out_of_range("TaskModel::getPlacementModel : PlacementModel index does not exist.");
  }
}

PlacementModel &TaskModel::getPlacementModel(const size_t index)
{
  // check the index value first
  if (index < placement_models_.size())
  {
    return placement_models_[index];
  } else
  {
    throw out_of_range("TaskModel::getPlacementModel : PlacementModel index does not exist.");
  }
}

void TaskModel::addPlacementModel(const PlacementModel &room)
{
  placement_models_.push_back(room);
}

void TaskModel::removePlacementModel(const size_t index)
{
  // check the index value first
  if (index < placement_models_.size())
  {
    placement_models_.erase(placement_models_.begin() + index);
  } else
  {
    throw out_of_range("TaskModel::removePlacementModel : PlacementModel index does not exist.");
  }
}

void TaskModel::getUniqueItems(vector<Item> &items) const
{
  // parse out names of items
  map<string, Item> unique_items;
  Pose zero_pose;
  for (size_t i = 0; i < placement_models_.size(); i++)
  {
    const Item &placement_item = placement_models_[i].getPlacement().getItem();
    const Object &placement_object = placement_models_[i].getPlacement().getObject();
    const Surface &placement_surface = placement_models_[i].getPlacement().getSurface();
    if (!placement_item.getName().empty() && unique_items.find(placement_item.getName()) == unique_items.end())
    {
      // add the new item to the list
      unique_items[placement_item.getName()] = placement_item;
      // don't assign a pose or frame ID
      unique_items[placement_item.getName()].setPose(zero_pose);
      unique_items[placement_item.getName()].setFrameID("");
    }
  }

  // now add the unique list to the vector
  for (map<string, Item>::iterator iter = unique_items.begin(); iter != unique_items.end(); ++iter)
  {
    items.push_back(iter->second);
  }
}

void TaskModel::getUniqueSurfaces(vector<Surface> &surfaces) const
{
  // parse out names of surfaces
  map<string, Surface> unique_surfaces;
  Pose zero_pose;
  for (size_t i = 0; i < placement_models_.size(); i++)
  {
    const Surface &placement_surface = placement_models_[i].getPlacement().getSurface();
    if (!placement_surface.getName().empty()
        && unique_surfaces.find(placement_surface.getName()) == unique_surfaces.end())
    {
      // add the new surface to the list
      unique_surfaces[placement_surface.getName()] = placement_surface;
      // don't assign a pose or frame ID
      unique_surfaces[placement_surface.getName()].setPose(zero_pose);
      unique_surfaces[placement_surface.getName()].setFrameID("");
    }
  }

  // now add the unique list to the vector
  for (map<string, Surface>::iterator iter = unique_surfaces.begin(); iter != unique_surfaces.end(); ++iter)
  {
    surfaces.push_back(iter->second);
  }
}
