/*!
 * \file Object.cpp
 * \brief Object configuration information.
 *
 * An object is an abstract entity in the world. Objects can be mutable entities (e.g., manipulable items) or
 * immutable entities (e.g., rooms, surfaces, and furniture). Objects can have an array of associated aliases
 * associated with their main name.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

// worldlib
#include "worldlib/world/Object.h"

// Boost
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::geometry;
using namespace rail::spatial_temporal_learning::worldlib::world;

Object::Object(const string &name, const string &frame_id, const Pose &pose, const double width, const double depth,
    const double height) : name_(name), frame_id_(frame_id), pose_(pose)
{
  width_ = width;
  depth_ = depth;
  height_ = height;
}

const string &Object::getName() const
{
  return name_;
}

void Object::setName(const string &name)
{
  name_ = name;
}

const string &Object::getFrameID() const
{
  return frame_id_;
}

void Object::setFrameID(const string &frame_id)
{
  frame_id_ = frame_id;
}

const Pose &Object::getPose() const
{
  return pose_;
}

Pose &Object::getPose()
{
  return pose_;
}

void Object::setPose(Pose &pose)
{
  pose_ = pose;
}

double Object::getWidth() const
{
  return width_;
}

void Object::setWidth(const double width)
{
  width_ = width;
}

double Object::getDepth() const
{
  return depth_;
}

void Object::setDepth(const double depth)
{
  depth_ = depth;
}

double Object::getHeight() const
{
  return height_;
}

void Object::setHeight(const double height)
{
  height_ = height;
}

const vector<string> &Object::getAliases() const
{
  return aliases_;
}

size_t Object::getNumAliases() const
{
  return aliases_.size();
}

const string &Object::getAlias(const size_t index) const
{
  // check the index value first
  if (index < aliases_.size())
  {
    return aliases_[index];
  } else
  {
    throw out_of_range("Object::getAlias : Alias index does not exist.");
  }
}

void Object::addAlias(const string &alias)
{
  aliases_.push_back(alias);
}

void Object::removeAlias(const size_t index)
{
  // check the index value first
  if (index < aliases_.size())
  {
    aliases_.erase(aliases_.begin() + index);
  } else
  {
    throw out_of_range("Object::removeAlias : Alias index does not exist.");
  }
}

bool Object::checkName(const string &name) const
{
  string name_uc = boost::to_upper_copy(name);

  // check the basic name
  if (boost::to_upper_copy(name_) == name_uc)
  {
    return true;
  } else
  {
    // check the aliases
    for (size_t i = 0; i < aliases_.size(); i++)
    {
      if (boost::to_upper_copy(aliases_[i]) == name_uc)
      {
        return true;
      }
    }
  }

  // no match found
  return false;
}

Position Object::fromParentFrame(const Position &position) const
{
  // create a Pose and use the Pose method
  Pose p1(position);
  Pose p2 = this->fromParentFrame(p1);
  return p2.getPosition();
}

Pose Object::fromParentFrame(const Pose &pose) const
{
  // use TF2
  tf2::Transform t_pose_parent = pose.toTF2Transform();
  tf2::Transform t_object_parent = pose_.toTF2Transform();
  tf2::Transform t_pose_object = t_object_parent.inverseTimes(t_pose_parent);
  return Pose(t_pose_object);
}
