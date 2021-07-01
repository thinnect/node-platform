/**
 * IEEE EUI-64 definitions header.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef EUI64_H_
#define EUI64_H_

#include <stdbool.h>
#include <stdint.h>

#define EUI64_LENGTH 8
#define IEEE_EUI64_LENGTH 8

typedef struct ieee_eui64 {
	uint8_t data[IEEE_EUI64_LENGTH];
} __attribute__((packed))ieee_eui64_t;

/**
 * Check if EUI64 has a value of all-zeros.
 * @param eui The EUI64 to check.
 * @return true if all zeros.
 */
bool eui64_is_zeros(const ieee_eui64_t * eui);

/**
 * Make the EUI64 have a value of all-zeros.
 * @param eui The EUI64 to modify.
 */
void eui64_set_zeros(ieee_eui64_t * eui);

/**
 * Check if EUI64 has a value of all-ones.
 * @param eui The EUI64 to check.
 * @return true if all ones.
 */
bool eui64_is_ones(const ieee_eui64_t * eui);

/**
 * Make the EUI64 have a value of all-ones.
 * @param eui The EUI64 to modify.
 */
void eui64_set_ones(ieee_eui64_t * eui);

void eui64_set(ieee_eui64_t * eui, const uint8_t data[IEEE_EUI64_LENGTH]);

void eui64_get(const ieee_eui64_t * eui, uint8_t data[IEEE_EUI64_LENGTH]);

int eui64_compare(const ieee_eui64_t * eui1, const ieee_eui64_t * eui2);

#endif// EUI64_H_
