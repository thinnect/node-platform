#ifndef NESC_TO_C_COMPAT_H_
#define NESC_TO_C_COMPAT_H_

#include <stdint.h>

#define nx_struct struct
typedef uint8_t nx_uint8_t;
typedef uint16_t nx_uint16_t;
typedef uint32_t nx_uint32_t;
typedef int8_t nx_int8_t;
typedef int16_t nx_int16_t;
typedef int32_t nx_int32_t;

typedef int64_t time64_t; // TODO this should be more general and come from elsewhere?
typedef int64_t nx_time64_t;

#endif//NESC_TO_C_COMPAT_H_
