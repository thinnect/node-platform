/**
 * A supposedly common and basic SPIFFS filesystem setup procedure.
 *
 * !!! Look at the code, make sure your use-case falls under the same
 *     understanding of basic setup. !!!
 *
 * Copyright Thinnect Inc. 2023
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

#define APPLICATION_PARTITION_NUMBER 2
#define APPLICATION_FILESYSTEM_NUMBER 0

/**
 * Read flash chip JEDEC identifier and print it out.
 * Initialize the spi_flash module.
 * Initialize filesystem to access a specific area of flash.
 * Start the filesystem module.
 */
void basic_rtos_filesystem_setup ()
{
    // SPI for dataflash
    RETARGET_SpiInit();

    // Wake flash from deep sleep
    spi_flash_resume();

    // Get dataflash chip ID
    uint8_t jedec[3] = {0};
    uint32_t flash_size;

    RETARGET_SpiTransferHalf(0, "\x9F", 1, jedec, 3);
    flash_size = (1 << jedec[2]);

    info1("Manufacturer ID: %02X", jedec[0]);
    info1("Mem Type: %02X", jedec[1]);
    info1("Mem Size: %u bytes", flash_size);

    if ((0x00 == jedec[0])||(0xFF == jedec[0])) // Invalid ID, flash not responsive
    {
        for (uint8_t i = 0; i < 10; i++)
        {
            err1("SPI flash");
            osDelay(1000);
        }
        PLATFORM_HardReset(); // A hard-reset may help on some boards
    }

    // Initialize flash subsystem
    spi_flash_init(flash_size);

    watchdog_feed();

    // debug1("Mass erase!");
    // spi_flash_mass_erase();

    m_spi_fs_driver.read = spi_flash_read;
    m_spi_fs_driver.write = spi_flash_write;
    m_spi_fs_driver.erase = spi_flash_erase;
    m_spi_fs_driver.size = spi_flash_size;
    m_spi_fs_driver.erase_size = spi_flash_erase_size;
    m_spi_fs_driver.suspend = spi_flash_suspend;
    m_spi_fs_driver.lock = spi_flash_lock;
    m_spi_fs_driver.unlock = spi_flash_unlock;

    fs_init(APPLICATION_FILESYSTEM_NUMBER, APPLICATION_PARTITION_NUMBER, &m_spi_fs_driver);

    watchdog_feed();
    fs_start(); // This can take several minutes if flash is uninitialized or corrupt.
    watchdog_feed();
}
