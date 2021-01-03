/**
 * Node coordinate APIs.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef NODE_COORDINATES_H_
#define NODE_COORDINATES_H_

#include "coordinates.h"

#include <stdbool.h>

/**
 * Get node geographic coordinates.
 *
 * @param geo Pointer to structure for storing coordinates.
 * @return true when coordinates were available and stored in geo.
 */
bool node_coordinates_get(coordinates_geo_t * geo);

#endif//NODE_COORDINATES_H_
