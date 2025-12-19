/*
 * Generic circular buffers.
 *
 * Copyright Thinnect Inc. 2025
 * @license MIT
 */
#ifndef CIRCULAR_BUFFER_H_
#define CIRCULAR_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t * buffer;
    uint8_t head;
    uint8_t tail;
    uint8_t maxlen;
} circular_buffer_u8_t;


void circular_buffer_u8_init(circular_buffer_u8_t * cb, uint8_t * buf, uint8_t length);

bool circular_buffer_u8_push(circular_buffer_u8_t * cb, uint8_t value);

bool circular_buffer_u8_pop(circular_buffer_u8_t * cb, uint8_t * value);

#endif//CIRCULAR_BUFFER_H_
