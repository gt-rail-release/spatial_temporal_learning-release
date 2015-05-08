/*!
 * \file worldlib.h
 * \brief All-inclusive include file for the worldlib library.
 *
 * The worldlib library is a standalone C++ library for world state learning methods.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_H_

// Geometry
#include "geometry/Orientation.h"
#include "geometry/Pose.h"
#include "geometry/Position.h"

// Model
#include "model/PersistenceModel.h"
#include "model/PlacementModel.h"
#include "model/TaskModel.h"

// Remote
#include "remote/Client.h"
#include "remote/HttpClient.h"
#include "remote/InteractiveWorldModelClient.h"
#include "remote/Node.h"
#include "remote/SpatialWorldClient.h"
#include "remote/SpatialWorldObservation.h"
#include "remote/SqlClient.h"
#include "remote/SqlEntity.h"

// World
#include "world/Item.h"
#include "world/Object.h"
#include "world/Observation.h"
#include "world/Placement.h"
#include "world/PlacementSurface.h"
#include "world/PointOfInterest.h"
#include "world/Room.h"
#include "world/Surface.h"
#include "world/World.h"

#endif
