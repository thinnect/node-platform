/*
 * Generic circular buffers.
 *
 * Copyright Thinnect Inc. 2025
 * @license MIT
 */

#include "circular_buffer.h"


void circular_buffer_u8_init(circular_buffer_u8_t * cb, uint8_t * buf, uint8_t length)
{
    cb->buffer = buf;
    cb->head = 0;
    cb->tail = 0;
    cb->maxlen = length;
}

bool circular_buffer_u8_push(circular_buffer_u8_t * cb, uint8_t value)
{
    bool overwrite = false;
    int8_t next = cb->head + 1;
    if (next == cb->maxlen)
    {
        next = 0;
    }

    if (next == cb->tail)
    {
        uint8_t ntail = cb->tail + 1;
        if (ntail == cb->maxlen)
        {
            cb->tail = 0;
        }
        else
        {
            cb->tail = ntail;
        }
        overwrite = true;
    }

    cb->buffer[cb->head] = value;
    cb->head = next;

    return overwrite;
}

bool circular_buffer_u8_pop(circular_buffer_u8_t * cb, uint8_t * value)
{
    if (cb->head == cb->tail)
    {
        return false; // No data
    }

    int8_t next = cb->tail + 1;
    if (next == cb->maxlen)
    {
        next = 0;
    }

    *value = cb->buffer[cb->tail];
    cb->tail = next;
    return true;
}
