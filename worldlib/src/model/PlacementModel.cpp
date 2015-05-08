/*!
 * \file PlacementModel.h
 * \brief Placement model information.
 *
 * A placement model contains information about the strength of an item in reference to some object.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 25, 2015
 */

// worldlib
#include "worldlib/model/PlacementModel.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::model;
using namespace rail::spatial_temporal_learning::worldlib::world;

PlacementModel::PlacementModel(const Placement &placement, const double decision_value, const double sigma_x,
    const double sigma_y, const double sigma_theta) : placement_(placement)
{
  decision_value_ = decision_value;
  sigma_x_ = sigma_x;
  sigma_y_ = sigma_y;
  sigma_theta_ = sigma_theta;
}

const Placement &PlacementModel::getPlacement() const
{
  return placement_;
}

Placement &PlacementModel::getPlacement()
{
  return placement_;
}

void PlacementModel::setPlacement(const Placement &placement)
{
  placement_ = placement;
}

double PlacementModel::getDecisionValue() const
{
  return decision_value_;
}

void PlacementModel::setDecisionValue(const double decision_value)
{

  decision_value_ = decision_value;
}

double PlacementModel::getSigmaX() const
{
  return sigma_x_;
}

void PlacementModel::setSigmaX(const double sigma_x)
{
  sigma_x_ = sigma_x;
}

double PlacementModel::getSigmaY() const
{
  return sigma_y_;
}

void PlacementModel::setSigmaY(const double sigma_y)
{
  sigma_y_ = sigma_y;
}

double PlacementModel::getSigmaTheta() const
{
  return sigma_theta_;
}

void PlacementModel::setSigmaTheta(const double sigma_theta)
{
  sigma_theta_ = sigma_theta;
}
