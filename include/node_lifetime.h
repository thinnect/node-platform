/**
 * Node lifetime data API.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef NODE_LIFETIME_H_
#define NODE_LIFETIME_H_

#include <stdint.h>

/**
 * Get node total uptime over its lifetime.
 * @return Node lifetime.
 */
uint32_t node_lifetime_seconds ();

/**
 * Get number of times the node has booted during its life.
 * @return Number of boots the node has done.
 */
uint32_t node_lifetime_boots ();

/**
 * Synchronize lifetime data to persistent storage. Call this occasionally.
 */
void node_lifetime_sync ();

#endif//NODE_LIFETIME_H_
