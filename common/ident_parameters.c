/**
 * Parameters identifying the firmware build for cases where they are needed,
 * but cannot be referenced from constants (library use).
 *
 * Copyright Thinnect Inc. 2020.
 * @license MIT
 */
#include <stdint.h>

#include "ident_parameters.h"

uint32_t ident_timestamp ()
{
    return IDENT_TIMESTAMP;
}
