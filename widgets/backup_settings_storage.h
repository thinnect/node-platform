/**
 * Store some settings backups somewhere in external flash.
 * @copyright RP 2025
 * @license MIT
 */

#ifndef BACKUP_SETTINGS_STORAGE_H_
#define BACKUP_SETTINGS_STORAGE_H_

#include <stdint.h>

uint8_t backup_radio_channel_get (void);

void backup_store (uint8_t radio_channel);

#endif//BACKUP_SETTINGS_STORAGE_H_
