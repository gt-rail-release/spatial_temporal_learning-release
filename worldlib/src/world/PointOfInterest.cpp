/*!
 * \file PointOfInterest.cpp
 * \brief Point of interest configuration information.
 *
 * A point of interest represents a feature of a surface. For example, the Surface object "Stove" might have multiple
 * points of interest representing each burner.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

// worldlib
#include "worldlib/world/PointOfInterest.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::world;

PointOfInterest::PointOfInterest(const string &name, const string &frame_id, const geometry::Pose &pose,
    const double width, const double depth, const double height) : Object(name, frame_id, pose, width, depth, height)
{
}
