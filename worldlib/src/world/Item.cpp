/*!
 * \file Item.cpp
 * \brief Item configuration information.
 *
 * An item represents a manipulable object within the world.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

// worldlib
#include "worldlib/world/Item.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::world;

Item::Item(const string &name, const string &frame_id, const geometry::Pose &pose, const double width,
    const double depth, const double height) : Object(name, frame_id, pose, width, depth, height)
{
}
