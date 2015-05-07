/*!
 * \file Surface.h
 * \brief Surface configuration information.
 *
 * A surface consists of a name with associated placement frames and points of interest.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_SURFACE_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_SURFACE_H_

// worldlib
#include "Object.h"
#include "PlacementSurface.h"
#include "PointOfInterest.h"

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace world
{

/*!
 * \class Surface
 * \brief Surface configuration information.
 *
 * A surface consists of a name with associated placement frames and points of interest.
 */
class Surface : public Object
{
public:
  /*!
   * \brief Create a new Surface.
   *
   * Create a new empty Surface with the given name, frame ID, Pose (in reference to the room's frame ID) and
   * dimensions.
   *
   * \param name The name of the Surface (defaults to the empty string).
   * \param frame_id The frame ID of the Surface (defaults to the empty string).
   * \param pose The Pose of the Surface with respect to the Room (defaults to 0 Pose).
   * \param width The width of the Surface (along the x-axis) (defaults to 0).
   * \param depth The width of the Surface (along the y-axis) (defaults to 0).
   * \param height The height of the Surface (along the z-axis) (defaults to 0).
   */
  Surface(const std::string &name = "", const std::string &frame_id = "", const geometry::Pose &pose = geometry::Pose(),
      const double width = 0, const double depth = 0, const double height = 0);

  /*!
   * \brief Placement surfaces value accessor (immutable).
   *
   * Get the placement surfaces of this Surface.
   *
   * \return The placement surfaces.
   */
  const std::vector<PlacementSurface> &getPlacementSurfaces() const;

  /*!
   * \brief Placement surfaces value accessor.
   *
   * Get the placement surfaces of this Surface.
   *
   * \return The placement surfaces.
   */
  std::vector<PlacementSurface> &getPlacementSurfaces();

  /*!
   * \brief Placement surfaces size accessor.
   *
   * Get the number of placement surfaces of this Surface.
   *
   * \return The number of placement surfaces of this Surface.
   */
  size_t getNumPlacementSurfaces() const;

  /*!
   * \brief PlacementSurface value accessor (immutable).
   *
   * Get the PlacementSurface of this Surface at the given index.
   *
   * \param i The index of the PlacementSurface to get.
   * \return The PlacementSurface at the given index.
   * \throws std::out_of_range Thrown if the PlacementSurface at the given index does not exist.
   */
  const PlacementSurface &getPlacementSurface(const size_t index) const;

  /*!
   * \brief PlacementSurface value accessor.
   *
   * Get the PlacementSurface of this Surface at the given index.
   *
   * \param i The index of the PlacementSurface to get.
   * \return The PlacementSurface at the given index.
   * \throws std::out_of_range Thrown if the PlacementSurface at the given index does not exist.
   */
  PlacementSurface &getPlacementSurface(const size_t index);

  /*!
   * \brief PlacementSurface adder.
   *
   * Add the PlacementSurface to this Surface.
   *
   * \param PlacementSurface The new PlacementSurface to add.
   */
  void addPlacementSurface(const PlacementSurface &placement_surface);

  /*!
   * \brief PlacementSurface remover.
   *
   * Remove the PlacementSurface at the given index. An invalid index results in no effect.
   *
   * \param i The index of the PlacementSurface to remove.
   * \throws std::out_of_range Thrown if the PlacementSurface at the given index does not exist.
   */
  void removePlacementSurface(const size_t index);

  /*!
   * \brief Check for the existence of a PlacementSurface.
   *
   * Check for the existence of a PlacementSurface in the Surface. This will also check the aliases.Case is not
   * important.
   *
   * \param name The name or alias of the PlacementSurface to find.
   * \throws std::out_of_range Thrown if no PlacementSurface with the given name exists.
   */
  bool placementSurfaceExists(const std::string &name) const;

  /*!
   * \brief PlacementSurface finder (immutable).
   *
   * Find a PlacementSurface with the given name. This will also check the aliases. Case is not important. If multiple
   * placement surfaces exist with the given name, the first placement surface is returned.
   *
   * \param name The name or alias of the PlacementSurface to find.
   * \throws std::out_of_range Thrown if no PlacementSurface with the given name exists.
   */
  const PlacementSurface &findPlacementSurface(const std::string &name) const;

  /*!
   * \brief PlacementSurface finder.
   *
   * Find a PlacementSurface with the given name. This will also check the aliases. Case is not important. If multiple
   * placement surfaces exist with the given name, the first placement surface is returned.
   *
   * \param name The name or alias of the PlacementSurface to find.
   * \throws std::out_of_range Thrown if no PlacementSurface with the given name exists.
   */
  PlacementSurface &findPlacementSurface(const std::string &name);

  /*!
   * \brief Closest PlacementSurface finder (immutable).
   *
   * Find the closest PlacementSurface to the given Position. If no placement surfaces exist, an exception is thrown.
   *
   * \param position The position (in reference to the surface's frame) to find the closest PlacementSurface to.
   * \throws std::out_of_range Thrown if no placement surfaces exist.
   */
  const PlacementSurface &findClosestPlacementSurface(const geometry::Position &position) const;

  /*!
   * \brief Closest PlacementSurface finder.
   *
   * Find the closest PlacementSurface to the given Position. If no placement surfaces exist, an exception is thrown.
   *
   * \param position The position (in reference to the surface's frame) to find the closest PlacementSurface to.
   * \throws std::out_of_range Thrown if no placement surfaces exist.
   */
  PlacementSurface &findClosestPlacementSurface(const geometry::Position &position);

  /*!
   * \brief Closest PlacementSurface index finder.
   *
   * Find the closest PlacementSurface index to the given Position. If no placement surfaces exist, an exception is
   * thrown.
   *
   * \param position The position (in reference to the surface's frame) to find the closest PlacementSurface to.
   * \return The index of the closest PlacementSurface.
   * \throws std::out_of_range Thrown if no placement surfaces exist.
   */
  size_t findClosestPlacementSurfaceIndex(const geometry::Position &position) const;

  /*!
   * \brief Points of interest value accessor (immutable).
   *
   * Get the points of interest of this Surface.
   *
   * \return The points of interest.
   */
  const std::vector<PointOfInterest> &getPointsOfInterest() const;

  /*!
   * \brief Points of interest value accessor.
   *
   * Get the points of interest of this Surface.
   *
   * \return The points of interest.
   */
  std::vector<PointOfInterest> &getPointsOfInterest();

  /*!
   * \brief Points of interest size accessor.
   *
   * Get the number of points of interest of this Surface.
   *
   * \return The number of points of interest of this Surface.
   */
  size_t getNumPointsOfInterest() const;

  /*!
   * \brief Points of interest value accessor (immutable).
   *
   * Get the PointOfInterest of this Surface at the given index.
   *
   * \param i The index of the PointOfInterest to get.
   * \return The PointOfInterest at the given index.
   * \throws std::out_of_range Thrown if the PointOfInterest at the given index does not exist.
   */
  const PointOfInterest &getPointOfInterest(const size_t index) const;

  /*!
   * \brief Points of interest value accessor.
   *
   * Get the PointOfInterest of this Surface at the given index.
   *
   * \param i The index of the PointOfInterest to get.
   * \return The PointOfInterest at the given index.
   * \throws std::out_of_range Thrown if the PointOfInterest at the given index does not exist.
   */
  PointOfInterest &getPointOfInterest(const size_t index);

  /*!
   * \brief PointOfInterest adder.
   *
   * Add the PointOfInterest to this Surface.
   *
   * \param poi The new PointOfInterest to add.
   */
  void addPointOfInterest(const PointOfInterest &poi);

  /*!
   * \brief PointOfInterest remover.
   *
   * Remove the PointOfInterest at the given index. An invalid index results in no effect.
   *
   * \param i The index of the PointOfInterest to remove.
   * \throws std::out_of_range Thrown if the PointOfInterest at the given index does not exist.
   */
  void removePointOfInterest(const size_t index);

  /*!
   * \brief Check for the existence of a PointOfInterest.
   *
   * Check for the existence of a PointOfInterest in the Surface. This will also check the aliases. Case is not
   * important.
   *
   * \param name The name or alias of the PointOfInterest to find.
   * \throws std::out_of_range Thrown if no PointOfInterest with the given name exists.
   */
  bool pointOfInterestExists(const std::string &name) const;

  /*!
   * \brief PointOfInterest finder (immutable).
   *
   * Find a PointOfInterest with the given name. This will also check the aliases. Case is not important. If multiple
   * points of interest exist with the given name, the first PointOfInterest is returned.
   *
   * \param name The name or alias of the PointOfInterest to find.
   * \throws std::out_of_range Thrown if no PointOfInterest with the given name exists.
   */
  const PointOfInterest &findPointOfInterest(const std::string &name) const;

  /*!
   * \brief PointOfInterest finder.
   *
   * Find a PointOfInterest with the given name. This will also check the aliases. Case is not important. If multiple
   * points of interest exist with the given name, the first PointOfInterest is returned.
   *
   * \param name The name or alias of the PointOfInterest to find.
   * \throws std::out_of_range Thrown if no PointOfInterest with the given name exists.
   */
  PointOfInterest &findPointOfInterest(const std::string &name);

private:
  /*! List of placement surfaces. */
  std::vector<PlacementSurface> placement_surfaces_;
  /*! List of points of interest. */
  std::vector<PointOfInterest> pois_;
};

}
}
}
}

#endif
