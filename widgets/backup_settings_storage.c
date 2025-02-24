/**
 * Store some settings backups somewhere in external flash.
 * @copyright RP 2025
 * @license MIT
 */

#include "backup_settings_storage.h"

#include "spi_flash.h"
#include "checksum.h"

#include "loglevels.h"
#define __MODUUL__ "bckp"
#define __LOG_LEVEL__ (LOG_LEVEL_backup_settings_storage & BASE_LOG_LEVEL)
#include "log.h"

#ifndef BACKUP_STORAGE_OFFSET
// Using the last page of the OTA communication area, normally not touched by bootloader
#define BACKUP_STORAGE_OFFSET (3*4096)
#endif//BACKUP_STORAGE_OFFSET

struct backup_settings_storage // 32 bytes
{
    char b;
    char c;
    char k;
    char p;
    uint8_t version;
    uint8_t channel;
    uint8_t reserved[22];
    uint32_t crc;
} __attribute((packed));


uint8_t backup_radio_channel_get (void)
{
    struct backup_settings_storage backup;

    spi_flash_lock();
    spi_flash_read(0, BACKUP_STORAGE_OFFSET, sizeof(backup), (uint8_t*)&backup);
    spi_flash_unlock();

    uint32_t crc = crc_32((const unsigned char*)&backup, sizeof(backup) - sizeof(uint32_t));
    if ((backup.b == 'b')
      &&(backup.c == 'c')
      &&(backup.k == 'k')
      &&(backup.p == 'p')
      &&(backup.crc == crc))
    {
        debug1("bckp found ch: %d", backup.channel);
        return backup.channel;
    }
    else
    {
        debug1("garbage %x != %x", crc, backup.crc);
    }
    return 0;
}


void backup_store (uint8_t radio_channel)
{
    struct backup_settings_storage backup = {.b = 'b', .c = 'c', .k = 'k', .p = 'p',
                                             .version = 1,
                                             .channel = radio_channel};
    backup.crc = crc_32((const unsigned char*)&backup, sizeof(backup) - sizeof(uint32_t));

    spi_flash_lock();
    spi_flash_erase(0, BACKUP_STORAGE_OFFSET, spi_flash_erase_size(0));
    spi_flash_write(0, BACKUP_STORAGE_OFFSET, sizeof(backup), (uint8_t*)&backup);
    spi_flash_unlock();

    debug1("stored %d %x", sizeof(backup), backup.crc);
}
