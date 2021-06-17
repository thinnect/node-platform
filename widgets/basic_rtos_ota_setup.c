/**
 * A supposedly common and basic OTA on rtos and multi-hop setup procedure.
 *
 * !!! Look at the code, make sure your use-case falls under the same
 *     understanding of basic setup. !!!
 *
 * Copyright Thinnect Inc. 2020
 * @license <PROPRIETARY>
 */
#include "basic_rtos_ota_setup.h"

// OTA updater
#include "updater.h"
#include "updater_fs.h"

// Get board UUID
#include "DeviceSignature.h"

#include "watchdog.h"

#include "loglevels.h"
#define __MODUUL__ "bota"
#define __LOG_LEVEL__ (LOG_LEVEL_basic_rtos_ota_setup & BASE_LOG_LEVEL)
#include "log.h"
#include "sys_panic.h"

static comms_layer_t * mp_beat_comm;
static comms_layer_t * mp_direct_comm;
static bool m_use_sleep_control;
static bool m_bstk_stopped;
static bool * mp_feed_watchdog;
static comms_sleep_controller_t m_radio_ctrl;

#define SLEEP_BLOCK_TIMEOUT_MS 50

static void radio_status_changed (comms_layer_t* comms, comms_status_t status, void* user)
{
    if (comms == mp_beat_comm)
    {
        if (COMMS_STOPPED == status)
        {
            m_bstk_stopped = true;
        }
        else if (COMMS_STARTED == status)
        {
            m_bstk_stopped = false;
        }
    }
}

static void ota_updater_active (bool on)
{
    if (NULL != mp_beat_comm)
    {
        if (on)
        {
            if (COMMS_STARTED == comms_status(mp_beat_comm))
            {
                debug1("bs stop");
                watchdog_feed();

                *mp_feed_watchdog = false; // Stop feeding watchdog while switching radio modes

                while (!m_bstk_stopped)
                {
                    comms_stop(mp_beat_comm, radio_status_changed, NULL);
                    osDelay(100);
                }

                if (m_use_sleep_control)
                {
                    comms_sleep_block(&m_radio_ctrl);
                }
                else
                {
                    if (COMMS_STARTED != comms_status(mp_direct_comm))
                    {
                        if (COMMS_SUCCESS != comms_start(mp_direct_comm, radio_status_changed, NULL))
                        {
                            err1("rdo start");
                            osDelay(1000);
                            sys_panic("ota rdo start");
                        }
                    }
                }

                while (COMMS_STARTED != comms_status(mp_direct_comm))
                {
                    osDelay(10);
                }

                debug1("ota rdo rdy");
                watchdog_feed();
                *mp_feed_watchdog = true;
            }
            else
            {
                warn1("bs alrdy stopped");
            }
        }
        else
        {
            debug1("ota over");
            *mp_feed_watchdog = false;

            if (COMMS_STOPPED == comms_status(mp_beat_comm))
            {
                if (m_use_sleep_control)
                {
                    comms_sleep_allow(&m_radio_ctrl);
                }

                while (m_bstk_stopped)
                {
                    comms_start(mp_beat_comm, radio_status_changed, NULL);
                    osDelay(100);
                }
                debug1("bs start");
            }
        }
    }
    else
    {
        debug1("OTA SH mode");
        if (on)
        {
            comms_sleep_block(&m_radio_ctrl);
        }
        else
        {
            comms_sleep_allow(&m_radio_ctrl);
        }
    }
}

void basic_rtos_ota_setup (comms_layer_t * p_beat_comm, comms_layer_t * p_direct_comm, bool sleep_control, bool * p_feed_watchdog)
{
	mp_beat_comm = p_beat_comm;
	mp_direct_comm = p_direct_comm;
	m_use_sleep_control = sleep_control;
    mp_feed_watchdog = p_feed_watchdog;

    m_bstk_stopped = false;

    uint8_t board_uuid[16];
    sigGetBoardUUID(board_uuid);
    debugb1("board", board_uuid, 16);

    if ((m_use_sleep_control)||(NULL == p_beat_comm))
    {
        if (COMMS_SUCCESS != comms_register_sleep_controller(mp_direct_comm, &m_radio_ctrl, radio_status_changed, NULL))
        {
            err1("rctrl");
        }
    }

    fs_update_init();

    updater_init(mp_direct_comm, board_uuid, ota_updater_active);
}
