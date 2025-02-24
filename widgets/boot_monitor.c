/**
 * Boot stats and signalling through backup RAM.
 * @copyright RP 2025
 * @license MIT
 */
#include "boot_monitor.h"

#include "platform.h"

#include "checksum.h"  // crc_32
#include "em_rmu.h"

#ifndef BURAM_SIZE_BYTES
#define BURAM_SIZE_BYTES 128
#endif//BURAM_SIZE_BYTES

#define BOOT_MONITOR_MAGIC_MARKER 0xB00F
#define BOOT_MONITOR_VERSION 1
#ifndef BOOT_MONITOR_BURAM_OFFSET
#define BOOT_MONITOR_BURAM_OFFSET (BURAM_SIZE_BYTES / 4)
#endif//BOOT_MONITOR_BURAM_OFFSET

#include "loglevels.h"
#define __MODUUL__ "bmon"
#define __LOG_LEVEL__ (LOG_LEVEL_boot_monitor & BASE_LOG_LEVEL)
#include "log.h"

typedef struct buram_boot_monitor
{
    uint32_t length;
    uint32_t magic_marker; // B00F
    uint32_t version;
    uint32_t boots;
    uint32_t uptime;
    uint32_t flags;
    uint32_t short_boots; // Boots that lasted less than X seconds
    uint32_t radio_channel; // For crash recovery / firmware update situations, hold previous channel info
    uint32_t crc;
} __attribute((packed)) buram_boot_monitor_t;


static buram_boot_monitor_t * mp_bootmoni = &(((buram_boot_monitor_t*)BURAM)[BOOT_MONITOR_BURAM_OFFSET]);
static uint32_t m_last_uptime_s = 0;
static uint32_t m_last_flags = 0;


static uint32_t calculate_buram_crc (void)
{
    return crc_32((uint8_t*)mp_bootmoni, sizeof(buram_boot_monitor_t) - sizeof(uint32_t));
}

static void add_buram_crc (void)
{
    mp_bootmoni->crc = calculate_buram_crc();
}

static bool check_buram_crc (void)
{
    return mp_bootmoni->crc == calculate_buram_crc();
}

void boot_monitor_init (void)
{
    bool clear_buram = false;
    uint32_t crc = crc_32((uint8_t*)&(BURAM->RET), BURAM_SIZE_BYTES - sizeof(uint32_t));
    debug1("BU-CRC32 %x", crc);
    if (check_buram_crc())
    {
        if ((mp_bootmoni->length == sizeof(buram_boot_monitor_t))
          &&(mp_bootmoni->magic_marker == BOOT_MONITOR_MAGIC_MARKER)
          &&(mp_bootmoni->version == BOOT_MONITOR_VERSION))
        {
            if (mp_bootmoni->uptime < BOOT_MONITOR_SHORT_BOOT_S)
            {
                mp_bootmoni->short_boots++;
            }
            else if (mp_bootmoni->uptime > BOOT_MONITOR_GOOD_BOOT_S) // Ran for some time
            {
                mp_bootmoni->short_boots = 0;
            }
            mp_bootmoni->boots++;
            m_last_uptime_s = mp_bootmoni->uptime;
            mp_bootmoni->uptime = 0;

            if (mp_bootmoni->flags & BOOT_MONITOR_FLAG_CLEAR)
            {
                clear_buram = true;
                m_last_flags = BOOT_MONITOR_FLAG_CLEAR;
            }
            else
            {
                m_last_flags = mp_bootmoni->flags;
            }
            mp_bootmoni->flags = 0;
        }
        else
        {
            debug1("urecognized %d %d %d", mp_bootmoni->length, mp_bootmoni->magic_marker, mp_bootmoni->version);
            clear_buram = true;
        }
    }
    else
    {
        debug1("bad crc");
        clear_buram = true;
        m_last_flags = BOOT_MONITOR_FLAG_BURAM_BAD_CRC;
    }

    if (clear_buram)
    {
        debug1("BU-CLR");
        mp_bootmoni->length = sizeof(buram_boot_monitor_t);
        mp_bootmoni->magic_marker = BOOT_MONITOR_MAGIC_MARKER;
        mp_bootmoni->version = BOOT_MONITOR_VERSION;
        mp_bootmoni->boots = 1;
        mp_bootmoni->uptime = 0;
        mp_bootmoni->short_boots = 0;
        mp_bootmoni->flags = 0;
        mp_bootmoni->radio_channel = 0;
    }
    add_buram_crc();
}


void boot_monitor_update (uint32_t uptime_s)
{
    mp_bootmoni->uptime = uptime_s;
    add_buram_crc();
}


uint32_t boot_monitor_boots (void)
{
    return mp_bootmoni->boots;
}


uint32_t boot_monitor_short_boots (void)
{
    return mp_bootmoni->short_boots;
}


uint32_t boot_monitor_last_uptime (void)
{
    return m_last_uptime_s;
}


uint32_t boot_monitor_last_flags (void)
{
    return m_last_flags;
}

uint32_t boot_monitor_flags (void)
{
    return mp_bootmoni->flags;
}

bool boot_monitor_get_flag (uint32_t flag)
{
    return 0 != (mp_bootmoni->flags & flag);
}

void boot_monitor_set_flag (uint32_t flag)
{
    mp_bootmoni->flags |= flag;
    add_buram_crc();
}

uint8_t boot_monitor_get_radio_channel(void)
{
    return mp_bootmoni->radio_channel;
}

void boot_monitor_set_radio_channel(uint8_t channel)
{
    mp_bootmoni->radio_channel = channel;
    add_buram_crc();
}
