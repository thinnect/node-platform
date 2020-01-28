/*
 * Memory allocation for storing panic info.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */

#include <stddef.h>

const char * g_panic_file = NULL;
int    g_panic_line = 0;
const char * g_panic_str = NULL;
