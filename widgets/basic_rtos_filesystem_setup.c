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
#include "em_usart.h"
#include "retargetspiconfig.h"

#include "watchdog.h"

static fs_driver_t m_spi_fs_driver;

#include "loglevels.h"
#define __MODUUL__ "fss"
#define __LOG_LEVEL__ (LOG_LEVEL_basic_rtos_filesystem_setup & BASE_LOG_LEVEL)
#include "log.h"

#define APPLICATION_PARTITION_NUMBER 2
#define APPLICATION_FILESYSTEM_NUMBER 0

/**
 * Initialize the spi_flash module.
 * Initialize filesystem to access a specific area of flash.
 * Start the filesystem module.
 */
void basic_rtos_filesystem_setup (void)
{
    // SPI for dataflash
    RETARGET_SpiInit();

	debug1("baudrate:%u", USART_BaudrateGet(RETARGET_SPI_UART));

    // Wake flash from deep sleep
    spi_flash_resume();

    // Initialize flash subsystem
    spi_flash_init(get_flash_size());

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

    uint8_t buffer[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    debugb1("BuffW:", &buffer[0], 16);
    spi_flash_write(2, 0, 16, &buffer[0]);
    spi_flash_read(2, 0, 16, &buffer[0]);
    debugb1("BuffR:", &buffer[0], 16);
}
