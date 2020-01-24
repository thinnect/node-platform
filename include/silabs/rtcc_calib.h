/**
 * An RTCC calibration solution developed compensate the lack of an accurate
 * low-frequency oscillator on the tsb0 device. The frequency of the LFRCO
 * is determined through a comparison of the RTCC running on the LFRCO and a
 * Timer running on HFXO. It does not actually configure the RTCC, so it is
 * possible to run the "calibration" also for LFXO and other boards, to perform
 * some comparisons and such.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef RTCC_CALIB_H_
#define RTCC_CALIB_H_

#include <stdint.h>

typedef void freq_set_f(uint32_t freq_hz);

/**
 * Perform a calibration immediately, return calibration value. Will run for
 * some time!
 * @return RTCC frequency in Hz.
 */
uint32_t rtcc_calibrate_now ();

/**
 * Configure periodic calibration.
 * NB: Do this only once before starting the OS!
 * @param period_ms Calibration period in milliseconds.
 * @param setfunc A function to call with the new calibration value.
 */
void rtcc_configure_periodic_calibration(uint32_t period_ms, freq_set_f * setfunc);

#endif//RTCC_CALIB_H_
