/**
 * Data structures for geo and UTM coordinates.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef COORDINATES_H_
#define COORDINATES_H_

#include <stdint.h>

typedef struct coordinates_geo_t {
	int32_t latitude;  // 10e6 degrees
	int32_t longitude; // 10e6 degrees
	int32_t elevation; // centimeters
	char    type;      // Source of coordinates, G - GPS, F - fixed, A - area (center) ...
} coordinates_geo_t;

typedef struct coordinates_utm_t {
	int32_t easting;   // centimeters
	int32_t northing;  // centimeters
	int32_t elevation; // centimeters
	uint8_t zone;      // UTM zone
	char    band;      // UTM latitude band
} coordinates_utm_t;

typedef struct orientation_deg_t {
	int32_t yaw;   // 10e3 degrees
	int32_t pitch; // 10e3 degrees
	int32_t roll;  // 10e3 degrees
} orientation_deg_t;

#endif // COORDINATES_H_
