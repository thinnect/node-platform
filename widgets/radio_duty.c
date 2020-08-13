/**
 * A very basic radio duty cycling component.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#include "radio_duty.h"

#include "cmsis_os2.h"

#include "loglevels.h"
#define __MODUUL__ "rduty"
#define __LOG_LEVEL__ (LOG_LEVEL_radio_duty & BASE_LOG_LEVEL)
#include "log.h"
#include "sys_panic.h"

static comms_sleep_controller_t m_radio_ctrl;
static osTimerId_t m_sleep_timer;
static osTimerId_t m_wakeup_timer;
static uint32_t m_sleep_time;
static uint32_t m_awake_time;

static void radio_status_changed (comms_layer_t* comms, comms_status_t status, void* user)
{
    // Actual status change is checked by polling during startup, but the callback is mandatory
}

static void sleep_timer_fired (void * arg)
{
    debug1("sleep");
    comms_sleep_allow(&m_radio_ctrl);
    osTimerStart(m_wakeup_timer, m_sleep_time);
}

static void wakeup_timer_fired (void * arg)
{
    debug1("wakeup");
    comms_sleep_block(&m_radio_ctrl);
    osTimerStart(m_sleep_timer, m_awake_time);
}

void radio_duty_init(comms_layer_t * comms, uint32_t sleep_ms, uint32_t awake_ms)
{
	m_sleep_time = sleep_ms;
	m_awake_time = awake_ms;

    // Add sleep controller for doing periodic wakeups
    if (COMMS_SUCCESS != comms_register_sleep_controller(comms, &m_radio_ctrl, radio_status_changed, NULL))
    {
        err1("rctrl");
    }

    m_sleep_timer = osTimerNew(&sleep_timer_fired, osTimerOnce, NULL, NULL);
    m_wakeup_timer = osTimerNew(&wakeup_timer_fired, osTimerOnce, NULL, NULL);

    // Wake up the radio for initial setup
    comms_sleep_block(&m_radio_ctrl);

    osTimerStart(m_sleep_timer, sleep_ms + awake_ms);
}
