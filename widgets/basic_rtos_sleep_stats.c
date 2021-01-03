/**
 * Gather and print RTOS sleep statistics.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */

#include "basic_rtos_sleep_stats.h"

#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>

#include "radio.h"
#include "lptsleep.h"
#include "spi_flash.h"

#include "cmsis_os2_ext.h"

#include "loglevels.h"
#define __MODUUL__ "slps"
#define __LOG_LEVEL__ (LOG_LEVEL_basic_rtos_sleep_stats & BASE_LOG_LEVEL)
#include "log.h"


static uint64_t m_st = 0; // Last cycle sleep time
static uint64_t m_rt = 0; // Last cycle radio time
static uint64_t m_ft = 0; // Last cycle flash time
static uint64_t m_ut = 0; // Last cycle uptime
static uint32_t m_lp = 0; // Last cycle packets
static uint32_t m_lb = 0; // Last cycle bytes


void basic_rtos_sleep_stats()
{
	uint64_t sleeptime = ulLowPowerSleepTime();
	uint64_t radiotime = radio_sleep_time();
	uint64_t flashtime = spi_flash_suspended_time();
	uint64_t uptime = osCounterGetMilli64();

	int tsleep = (int)(100*sleeptime/uptime); // sleep since boot
	int lsleep = (int)(100*(sleeptime - m_st)/(uptime - m_ut)); // sleep in last interval
	info1("sleep %"PRIu64"/%"PRIu64" %d%% (%d%%)", sleeptime, uptime, tsleep, lsleep);

	int trsleep = (int)(100*radiotime/uptime); // radio sleep since boot
	int lrsleep = (int)(100*(radiotime - m_rt)/(uptime - m_ut)); // radio sleep in last interval
	info2("rsleep %"PRIu64"/%"PRIu64" %d%% (%d%%)", radiotime, uptime, trsleep, lrsleep);

	int tfsleep = (int)(100*flashtime/uptime); // flash sleep since boot
	int lfsleep = (int)(100*(flashtime - m_ft)/(uptime - m_ut)); // flash sleep in last interval
	info3("fsleep %"PRIu64"/%"PRIu64" %d%% (%d%%)", flashtime, uptime, tfsleep, lfsleep);

	m_ut = uptime;
	m_rt = radiotime;
	m_ft = flashtime;
	m_st = sleeptime;

	uint32_t txpackets = radio_tx_packets();
	uint32_t txbytes = radio_tx_bytes();
	info4("packets:%"PRIu32"(%"PRIu32") bytes:%"PRIu32"(%"PRIu32")",
	    txpackets, (uint32_t)(txpackets - m_lp),
	    txbytes, (uint32_t)(txbytes - m_lb));
	m_lp = txpackets;
	m_lb = txbytes;
}
