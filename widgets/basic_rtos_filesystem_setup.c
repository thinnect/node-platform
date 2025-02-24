/**
 * A supposedly common and basic SPIFFS filesystem setup procedure.
 *
 * !!! Look at the code, make sure your use-case falls under the same
 *     understanding of basic setup. !!!
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#include "basic_rtos_filesystem_setup.h"

#include "cmsis_os2.h"

#include "platform.h"
#include "retargetspi.h"
#include "spi_flash.h"
#include "fs.h"

#include "watchdog.h"

static fs_driver_t m_spi_fs_driver;

#include "loglevels.h"
#define __MODUUL__ "fss"
#define __LOG_LEVEL__ (LOG_LEVEL_basic_rtos_filesystem_setup & BASE_LOG_LEVEL)
#include "log.h"

/**
 * Read flash chip JEDEC identifier and print it out.
 * Initialize the spi_flash module.
 * Initialize filesystem to access a specific area of flash.
 * Start the filesystem module.
 */
bool basic_rtos_filesystem_setup ()
{
    // SPI for dataflash
    RETARGET_SpiInit();

    // Initialize flash subsystem and wake flash from deep sleep
    spi_flash_init();

    // Get dataflash chip ID
    uint8_t jedec[4] = {0};
    RETARGET_SpiTransferHalf(0, "\x9F", 1, jedec, 4);
    info1("jedec %02x%02x%02x%02x", jedec[0], jedec[1], jedec[2], jedec[3]);

    if ((0x00 == jedec[0])||(0xFF == jedec[0])) // Invalid ID, flash not responsive
    {
        for (uint8_t i = 0; i < 10; i++)
        {
            err1("SPI flash");
            osDelay(1000);
        }
        PLATFORM_HardReset(); // A hard-reset may help on some boards
    }

    // spi_flash_mass_erase();

    m_spi_fs_driver.read = spi_flash_read;
    m_spi_fs_driver.write = spi_flash_write;
    m_spi_fs_driver.erase = spi_flash_erase;
    m_spi_fs_driver.size = spi_flash_size;
    m_spi_fs_driver.erase_size = spi_flash_erase_size;
    m_spi_fs_driver.suspend = spi_flash_suspend;
    m_spi_fs_driver.lock = spi_flash_lock;
    m_spi_fs_driver.unlock = spi_flash_unlock;

    // TODO Non-magic numbers SPIFFS is partition 2 (third partition) and filesystem 0?
    fs_init(0, 2, &m_spi_fs_driver);

    watchdog_feed();
    fs_start(); // This can take several minutes if flash is uninitialized or corrupt.

    uint32_t fswaits = 0;
    while (! fs_ready())
    {
        osDelay(1000);
        watchdog_feed();
        info1("fs mounting...%d", fswaits);
        if (++fswaits > 30*6) // 30 minutes
        {
            err1("fs timeout");
            return false;
        }
    }
    return true;
}
