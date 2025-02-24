/**
 * Boot stats and signalling through backup RAM.
 * @copyright RP 2025
 * @license MIT
 */
#ifndef BOOT_MONITOR_H_
#define BOOT_MONITOR_H_

#include <stdint.h>
#include <stdbool.h>

#define BOOT_MONITOR_SHORT_BOOT_S 120
#define BOOT_MONITOR_GOOD_BOOT_S 3600

#define BOOT_MONITOR_FLAG_BURAM_BAD_CRC   (1 << 0)
#define BOOT_MONITOR_FLAG_CLEAR           (1 << 1)
#define BOOT_MONITOR_FLAG_RESERVED2       (1 << 2)
#define BOOT_MONITOR_FLAG_RESERVED3       (1 << 3)
#define BOOT_MONITOR_FLAG_FORCE_SAFE_MODE (1 << 4)
#define BOOT_MONITOR_FLAG_SAFE_MODE       (1 << 5)

void boot_monitor_init (void);

void boot_monitor_update (uint32_t uptime_s);

uint32_t boot_monitor_boots (void);

uint32_t boot_monitor_flags (void);

uint32_t boot_monitor_short_boots (void);

uint32_t boot_monitor_last_uptime (void);

uint32_t boot_monitor_last_flags (void);

bool boot_monitor_get_flag (uint32_t flag);
void boot_monitor_set_flag (uint32_t flag);

uint8_t boot_monitor_get_radio_channel (void);
void boot_monitor_set_radio_channel (uint8_t channel);

#endif//BOOT_MONITOR_H_
