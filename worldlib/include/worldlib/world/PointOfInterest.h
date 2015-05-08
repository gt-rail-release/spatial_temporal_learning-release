/*!
 * \file PointOfInterest.h
 * \brief Point of interest configuration information.
 *
 * A point of interest represents a feature of a surface. For example, the Surface object "Stove" might have multiple
 * points of interest representing each burner.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_POINT_OF_INTEREST_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_POINT_OF_INTEREST_H_

// worldlib
#include "Object.h"

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace world
{

/*!
 * \class PointOfInterest
 * \brief Point of interest configuration information.
 *
 * A point of interest represents a feature of a surface. For example, the Surface object "Stove" might have multiple
 * points of interest representing each burner.
 */
class PointOfInterest : public Object
{
public:
  /*!
   * \brief Create a new PointOfInterest.
   *
   * Create a new empty PointOfInterest with the given name, frame ID, Pose (in reference to the surfaces' frame ID)
   * and dimensions.
   *
   * \param name The name of the PlacementSurface (defaults to the empty string).
   * \param frame_id The frame ID of the PlacementSurface (defaults to the empty string).
   * \param pose The Pose of the PlacementSurface with respect to the Surface (defaults to 0 Pose).
   * \param width The width of the PlacementSurface (along the x-axis) (defaults to 0).
   * \param depth The width of the PlacementSurface (along the y-axis) (defaults to 0).
   * \param height The height of the PlacementSurface (along the z-axis) (defaults to 0).
   */
  PointOfInterest(const std::string &name = "", const std::string &frame_id = "",
      const geometry::Pose &pose = geometry::Pose(), const double width = 0, const double depth = 0,
      const double height = 0);
};

}
}
}
}

#endif
