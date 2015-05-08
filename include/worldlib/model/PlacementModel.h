/*!
 * \file PlacementModel.h
 * \brief Placement model information.
 *
 * A placement model contains information about the strength of an item in reference to some object.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 25, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_MODEL_PLACEMENT_MODEL_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_MODEL_PLACEMENT_MODEL_H_

// worldlib
#include "../world/Placement.h"

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace model
{

/*!
 * \class PlacementModel
 * \brief Placement model information.
 *
 * A placement model contains information about the strength of an item in reference to some object.
 */
class PlacementModel
{
public:
  /*!
   * \brief Create a new PlacementModel.
   *
   * Create a new PlacementModel with the given parameters.
   *
   * \param placement The Placement for this PlacementModel.
   * \param decision_value The decision value for this PlacementModel.
   * \param sigma_x The standard deviation in the x-coordinate for this PlacementModel.
   * \param sigma_y The standard deviation in the y-coordinate for this PlacementModel.
   * \param sigma_theta The standard deviation in the angle (about the z-axis) for this PlacementModel.
   */
  PlacementModel(const world::Placement &placement, const double decision_value, const double sigma_x,
      const double sigma_y, const double sigma_theta);

  /*!
   * \brief Placement value accessor (immutable).
   *
   * Get the Placement value of this PlacementModel.
   *
   * \return The Placement value of this PlacementModel.
   */
  const world::Placement &getPlacement() const;

  /*!
   * \brief Placement value accessor.
   *
   * Get the Placement value of this PlacementModel.
   *
   * \return The Placement value of this PlacementModel.
   */
  world::Placement &getPlacement();

  /*!
   * \brief Placement value mutator.
   *
   * Set the Placement value of this PlacementModel.
   *
   * \param placement The new Placement value of this PlacementModel.
   */
  void setPlacement(const world::Placement &placement);

  /*!
   * \brief Decision value accessor.
   *
   * Get the decision value of this PlacementModel.
   *
   * \return The decision value of this PlacementModel.
   */
  double getDecisionValue() const;

  /*!
   * \brief Decision value mutator.
   *
   * Set the decision value of this PlacementModel.
   *
   * \param decision_value The new decision value of this PlacementModel.
   */
  void setDecisionValue(const double decision_value);

  /*!
   * \brief Sigma-x value accessor.
   *
   * Get the standard deviation along the x-axis value of this PlacementModel.
   *
   * \return The sigma-x value of this PlacementModel.
   */
  double getSigmaX() const;

  /*!
   * \brief Sigma-x value mutator.
   *
   * Set the standard deviation along the x-axis value of this PlacementModel.
   *
   * \param sigma_x The new sigma-x value of this PlacementModel.
   */
  void setSigmaX(const double sigma_x);

  /*!
   * \brief Sigma-y value accessor.
   *
   * Get the standard deviation along the y-axis value of this PlacementModel.
   *
   * \return The sigma-y value of this PlacementModel.
   */
  double getSigmaY() const;

  /*!
   * \brief Sigma-y value mutator.
   *
   * Set the standard deviation along the y-axis value of this PlacementModel.
   *
   * \param sigma_y The new sigma-y value of this PlacementModel.
   */
  void setSigmaY(const double sigma_y);

  /*!
   * \brief Sigma-theta value accessor.
   *
   * Get the standard deviation in the theta value of this PlacementModel.
   *
   * \return The sigma-theta value of this PlacementModel.
   */
  double getSigmaTheta() const;

  /*!
   * \brief Sigma-theta value mutator.
   *
   * Set the standard deviation in the theta value of this PlacementModel.
   *
   * \param sigma_theta The new sigma-theta value of this PlacementModel.
   */
  void setSigmaTheta(const double sigma_theta);

private:
  /*! The Placement value for the model. */
  world::Placement placement_;
  /*! The numeric attributes of this model. */
  double decision_value_, sigma_x_, sigma_y_, sigma_theta_;
};

}
}
}
}

#endif
