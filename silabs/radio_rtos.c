/*
 * Mist-comm compatible SiLabs RAIL based radio layer for CMSIS.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Veiko Rütter, Konstantin Bilozor, Raido Pahtma
 */
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <assert.h>

#include "radio.h"

#include "em_core.h"
#include "rail.h"
#include "rail_ieee802154.h"
#include "rail_types.h"
#include "rail_chip_specific.h"
#include "rail_assert_error_codes.h"
#include "pa_conversions_efr32.h"
#include "pa_curves_efr32.h"
#include "sleep.h"

#ifdef RAIL_USE_CUSTOM_CONFIG
// rail_config can be generated with SimplicityStudio, but it is not commonly
// necessary as an 802.15.4 standard conf is present in RAIL by default
#include "rail_config.h"
#endif//RAIL_USE_CUSTOM_CONFIG

#ifndef RADIO_INTERRUPT_PRIORITY
#error "RADIO_INTERRUPT_PRIORITY not defined - must be numerically >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY"
#endif//RADIO_INTERRUPT_PRIORITY

#ifndef DEFAULT_RAIL_TX_POWER_MODE
#define DEFAULT_RAIL_TX_POWER_MODE 'N'
#endif//DEFAULT_RAIL_TX_POWER_MODE

#if DEFAULT_RAIL_TX_POWER_MODE=='L'
    #pragma message "RAIL_TX_POWER_MODE_2P4GIG_LP"
    #define DEFAULT_RAIL_TX_POWER_MODE_2P4GIG RAIL_TX_POWER_MODE_2P4GIG_LP
#elif DEFAULT_RAIL_TX_POWER_MODE=='M'
    #pragma message "RAIL_TX_POWER_MODE_2P4GIG_MP"
    #define DEFAULT_RAIL_TX_POWER_MODE_2P4GIG RAIL_TX_POWER_MODE_2P4GIG_MP
#elif DEFAULT_RAIL_TX_POWER_MODE=='H'
    #pragma message "RAIL_TX_POWER_MODE_2P4GIG_HP"
    #define DEFAULT_RAIL_TX_POWER_MODE_2P4GIG RAIL_TX_POWER_MODE_2P4GIG_HP
#else
    #error Select default PA by defining CFLAGS+=-DDEFAULT_RAIL_TX_POWER_MODE="'[H/M/L]'", note the proper use of quotes (="'X'").
#endif

#include "cmsis_os2.h"

#include "mist_comm_iface.h"
#include "mist_comm_am.h"

#include "radio_seqNum.h"

#include "loglevels.h"
#define __MODUUL__ "radio"
#define __LOG_LEVEL__ (LOG_LEVEL_radio & BASE_LOG_LEVEL)
#include "log.h"
#include "sys_panic.h" // Also makes use of __MODUUL__

// queue -----------------------------------------------------------------------
typedef struct radio_queue_element radio_queue_element_t;
struct radio_queue_element
{
    comms_msg_t* msg;
    comms_send_done_f *send_done;
    void *user;
    uint32_t timestamp_queued;
    radio_queue_element_t* next;
};
// -----------------------------------------------------------------------------

#define RADIO_MAX_SEND_TIME_MS 1000UL

// Thread flag definitions
#define RDFLG_RADIO_DEINIT        (1 << 0)
#define RDFLG_RADIO_START         (1 << 1)
#define RDFLG_RADIO_STOP          (1 << 2)
#define RDFLG_RADIO_RESTART       (1 << 3)
#define RDFLG_RADIO_SEND          (1 << 4)
#define RDFLG_RADIO_RESEND        (1 << 5)
#define RDFLG_RADIO_SEND_TIMEOUT  (1 << 6)
#define RDFLG_RADIO_SEND_FAIL     (1 << 7)

#define RDFLG_RAIL_SEND_DONE      (1 << 8)
#define RDFLG_RAIL_SEND_BUSY      (1 << 9)
#define RDFLG_RAIL_SEND_FAIL      (1 << 10)
#define RDFLG_RAIL_RX_BUSY        (1 << 11)
#define RDFLG_RAIL_RX_SUCCESS     (1 << 12)
#define RDFLG_RAIL_RX_OVERFLOW    (1 << 13)
#define RDFLG_RAIL_RX_FRAME_ERROR (1 << 14)
#define RDFLG_RAIL_RX_ABORT       (1 << 15)
#define RDFLG_RAIL_RX_FAIL        (1 << 16)
#define RDFLG_RAIL_TXACK_SENT     (1 << 17)
#define RDFLG_RAIL_RXACK_TIMEOUT  (1 << 18)
#define RDFLG_RAIL_RX_MORE        (1 << 20)

#define RDFLGS_ALL                (0x7FFFFFFF)

typedef enum RadioState
{
    ST_UNINITIALIZED,
    ST_OFF,
    ST_STARTING,
    ST_RUNNING,
    ST_STOPPING
} RadioState_t;

volatile int g_rail_invalid_actions = 0;

static uint16_t m_radio_address;
static uint16_t m_radio_pan_id;
static uint8_t m_radio_channel_configured;
static uint8_t m_radio_channel_current;

static RAIL_TxPowerMode_t m_rail_power_mode = DEFAULT_RAIL_TX_POWER_MODE_2P4GIG;
static int16_t m_rail_tx_power = DEFAULT_RFPOWER_DBM * 10; // RAIL uses deci-dBm

static comms_layer_am_t m_radio_iface;

static RAIL_Handle_t m_rail_handle;
static RAIL_Status_t m_rx_fifo_status;
static uint8_t m_radio_tx_num;
static bool radio_tx_wait_ack;

static RadioState_t m_state = ST_UNINITIALIZED;
static bool m_radio_busy;

// Radio test mode options
static RAIL_StreamMode_t m_stream_mode;
static bool m_stream_mode_enabled;
static bool m_stream_mode_active;

// Diagnostic counters
static volatile uint8_t rx_busy;
static volatile uint8_t rx_overflow;
static volatile uint8_t rx_frame_error;
static volatile uint8_t rx_abort;
static volatile uint8_t rx_fail;
static volatile uint8_t tx_ack_sent;

// TX time monitoring variables
static uint32_t m_radio_send_timestamp;
static uint32_t m_rail_send_timestamp;
static uint32_t m_rail_sent_timestamp;

// CSMA retries
static uint8_t m_csma_retries;

// Time spent if OFF state since radio was initialized
static uint32_t m_sleep_time;

// When the radio was stopped last
static uint32_t m_stop_timestamp;

// Packets actually transmitted (energy for TX spent)
static uint32_t m_transmitted_packets;

// Bytes actually transmitted (energy for TX spent)
static uint32_t m_transmitted_bytes;

// Internal RAIL initialization procedures
static RAIL_Handle_t radio_rail_init();

// The main RAIL callback function
static void radio_rail_event_cb(RAIL_Handle_t m_rail_handle, RAIL_Events_t events);

static uint32_t radio_timestamp();

// OS timer callbacks
static void radio_send_timeout_cb(void* argument);
static void radio_resend_timeout_cb(void* argument);

// Start-stop callbacks
static comms_status_change_f * m_state_change_cb;
static void * m_state_change_user;

// Functions to wire into the API
static comms_error_t radio_start (comms_layer_iface_t * iface,
                                  comms_status_change_f * start_done, void * user);
static comms_error_t radio_stop  (comms_layer_iface_t * iface,
                                  comms_status_change_f * stop_done, void * user);
static comms_error_t radio_send  (comms_layer_iface_t * iface,
                                  comms_msg_t * msg,
                                  comms_send_done_f * send_done, void * user);

// TX queue with a linked list -------------------------------------------------
static radio_queue_element_t radio_msg_queue_memory[7];
static radio_queue_element_t* radio_msg_queue_free;
static radio_queue_element_t* radio_msg_queue_head;
static radio_queue_element_t* radio_msg_sending;
// -----------------------------------------------------------------------------

// RX queue with os message queue ----------------------------------------------
osMessageQueueId_t m_rx_queue;
// -----------------------------------------------------------------------------

// Radio threading, timers -----------------------------------------------------
static void radio_thread(void * p);

static osThreadId_t m_radio_thread_id;
static osMutexId_t m_radio_mutex;

static osTimerId_t m_send_timeout_timer;
static osTimerId_t m_resend_timer;
// -----------------------------------------------------------------------------


static void radio_rail_rfready_cb(RAIL_Handle_t m_rail_handle)
{
    // No actions taken right now
}


static void radio_rail_config_changed_cb(RAIL_Handle_t m_rail_handle,
                                         const RAIL_ChannelConfigEntry_t *entry)
{
    // No actions taken right now
}


// General radio configuration, may be tweaked between init and start
static RAIL_IEEE802154_Config_t m_radio_ieee802154_config = {
    .addresses = NULL,
    .ackConfig = {
        .enable = true,
        .ackTimeout = 864, // 54 symbols * 16 us/symbol = 864 us.
        .rxTransitions = {
            .success = RAIL_RF_STATE_RX,
            .error = RAIL_RF_STATE_RX // ignored
        },
        .txTransitions = {
            .success = RAIL_RF_STATE_RX,
            .error = RAIL_RF_STATE_RX // ignored
        }
    },
    .timings = {
        .idleToTx = 100,
        .idleToRx = 100,
        .rxToTx = 192, // 192
        // Make txToRx slightly lower than desired to make sure we get to RX in time
        .txToRx = 192 - 10, // 192 - 10
        .rxSearchTimeout = 0,
        .txToRxSearchTimeout = 0
    },
    .framesMask = RAIL_IEEE802154_ACCEPT_STANDARD_FRAMES,// | RAIL_IEEE802154_ACCEPT_ACK_FRAMES,
    .promiscuousMode = false,
    .isPanCoordinator = false
};


comms_layer_t* radio_init (uint16_t channel, uint16_t pan_id, uint16_t address)
{
    m_radio_channel_configured = channel;
    m_radio_pan_id = pan_id;
    m_radio_address = address;

    m_radio_tx_num = 0;

    m_sleep_time = 0;
    m_stop_timestamp = 0;

    m_transmitted_packets = 0;
    m_transmitted_bytes = 0;

    radio_msg_sending = NULL;
    radio_msg_queue_head = NULL;
    radio_msg_queue_free = &radio_msg_queue_memory[0];
    radio_msg_queue_free->next = NULL;
    for(uint8_t i=1;i<sizeof(radio_msg_queue_memory)/sizeof(radio_queue_element_t);i++)
    {
        radio_msg_queue_memory[i].next = (radio_queue_element_t*)radio_msg_queue_free;
        radio_msg_queue_free = &radio_msg_queue_memory[i];
    }

    m_rx_queue = osMessageQueueNew(10, sizeof(RAIL_RxPacketHandle_t), NULL);
    if (m_rx_queue == NULL)
    {
        err1("rxq");
        return(NULL);
    }

    m_radio_mutex = osMutexNew(NULL);
    m_send_timeout_timer = osTimerNew(&radio_send_timeout_cb, osTimerOnce, NULL, NULL);
    m_resend_timer = osTimerNew(&radio_resend_timeout_cb, osTimerOnce, NULL, NULL);

    comms_am_create((comms_layer_t *)&m_radio_iface, m_radio_address, radio_send, radio_start, radio_stop);

    const osThreadAttr_t radio_thread_attr = { .name = "radio" };
    m_radio_thread_id = osThreadNew(radio_thread, NULL, &radio_thread_attr);

    m_state = ST_OFF; // Radio initialized, but not turned ON yet
    m_radio_busy = false; // Protected busy state

    m_stream_mode_enabled = false;
    m_stream_mode_active = false;

    info1("channel %d pan %02X rfpwr %d",
        (int)channel, (int)pan_id,
        (int)DEFAULT_RFPOWER_DBM);

    return (comms_layer_t *)&m_radio_iface;
}

void radio_deinit (comms_layer_t * iface)
{
    if (iface == (comms_layer_t*)&m_radio_iface)
    {
        osThreadFlagsSet(m_radio_thread_id, RDFLG_RADIO_DEINIT);
        // osThreadJoin not implented in all CMSIS adapters (FreeRTOS)
        while (osThreadTerminated != osThreadGetState(m_radio_thread_id))
        {
            osThreadYield(); // Wait for termination ...
        }
    }
}


// Configure radio promiscuous mode, will take effect after radio is restarted
void radio_set_promiscuous (bool promiscuous)
{
    m_radio_ieee802154_config.promiscuousMode = promiscuous;
}


// Set radio channel
bool radio_set_channel (uint16_t channel)
{
    if ((channel >= 11) && (channel <= 26))
    {
        m_radio_channel_configured = channel;
        return true;
    }
    return false;
}

// Get radio channel.
uint16_t radio_get_channel ()
{
    return m_radio_channel_current;
}


// Select radio PA, will take effect after radio is restarted
bool radio_set_power_mode (int pa)
{
    switch((RAIL_TxPowerMode_t)pa)
    {
        case RAIL_TX_POWER_MODE_2P4GIG_HP:
        break;
    #ifdef RAIL_TX_POWER_MODE_2P4GIG_MP
        case RAIL_TX_POWER_MODE_2P4GIG_MP:
        break;
    #endif//RAIL_TX_POWER_MODE_2P4GIG_MP
        case RAIL_TX_POWER_MODE_2P4GIG_LP:
        break;
        default:
            return false;
    }

    m_rail_power_mode = (RAIL_TxPowerMode_t)pa;
    return true;
}

// Set radio TX power, will take effect after radio is restarted
void radio_set_tx_power (float pwr)
{
    m_rail_tx_power = (int16_t)(pwr * 10);
}


// Configure radio for test stream mode, will take effect after radio is restarted
bool radio_stream_mode_enable (int mode)
{
    if(mode < RAIL_STREAM_MODES_COUNT)
    {
        m_stream_mode_enabled = true;
        m_stream_mode = (RAIL_StreamMode_t)mode;
        return true;
    }
    return false;
}

// Disable radio test stream mode, will take effect after radio is restarted
void radio_stream_mode_disable ()
{
    m_stream_mode_enabled = false;
}


/**
 * Configure RAIL.
 */
static RAIL_Handle_t radio_rail_init ()
{
    RAIL_Handle_t handle;

    //RAIL_DECLARE_TX_POWER_VBAT_CURVES(piecewiseSegments, curvesSg, curves24Hp, curves24Lp);

    static RAIL_Config_t rail_config = {
        .eventsCallback = &radio_rail_event_cb
    };

    rx_abort = 0;
    rx_busy = 0;
    rx_overflow = 0;
    rx_fail = 0;
    m_rx_fifo_status = RAIL_STATUS_NO_ERROR-1;

    int32_t priority = RADIO_INTERRUPT_PRIORITY; // not shifted
    NVIC_SetPriority(FRC_PRI_IRQn, priority);
    NVIC_SetPriority(FRC_IRQn, priority);
    NVIC_SetPriority(MODEM_IRQn, priority);
    NVIC_SetPriority(RAC_SEQ_IRQn, priority);
    NVIC_SetPriority(RAC_RSM_IRQn, priority);
    NVIC_SetPriority(BUFC_IRQn, priority);
    NVIC_SetPriority(AGC_IRQn, priority);
    NVIC_SetPriority(PROTIMER_IRQn, priority);
    NVIC_SetPriority(SYNTH_IRQn, priority);
    #ifdef RFSENSE_IRQn
    NVIC_SetPriority(RFSENSE_IRQn, priority); // Not supported on Series2 ?
    #endif//RFSENSE_IRQn
    #ifdef PRORTC_IRQn
    NVIC_SetPriority(PRORTC_IRQn, priority); // Not supported on some chips ?
    #endif

    handle = RAIL_Init(&rail_config, &radio_rail_rfready_cb);
    if (NULL == handle)
    {
        err1("RAIL INIT ERROR");
    }
    return handle;
}


static RAIL_Status_t radio_rail_configure (RAIL_Handle_t handle)
{
    RAIL_Status_t rs;

    RAIL_DECLARE_TX_POWER_VBAT_CURVES_ALT;
    static const RAIL_TxPowerCurvesConfigAlt_t txPowerCurvesConfig = RAIL_DECLARE_TX_POWER_CURVES_CONFIG_ALT;

    // Put the variables declared above into the appropriate structure
    //RAIL_TxPowerCurvesConfig_t txPowerCurvesConfig = { curves24Hp, curvesSg, curves24Lp, piecewiseSegments };

    // In the Silicon Labs implementation, the user is required to save those curves into
    // to be referenced when the conversion functions are called
    //RAIL_InitTxPowerCurves(&txPowerCurvesConfig);
    RAIL_InitTxPowerCurvesAlt(&txPowerCurvesConfig);

    // Enabling will ensure that the PA power remains constant chip-to-chip.
    RAIL_EnablePaCal(true);

    // Declare the structure used to configure the PA
    // Battery: { RAIL_TX_POWER_MODE_2P4_HP, 1800, 10 };

    static RAIL_TxPowerConfig_t txPowerConfig = { DEFAULT_RAIL_TX_POWER_MODE_2P4GIG, 3300, 10 };
    txPowerConfig.mode = m_rail_power_mode;

    rs = RAIL_ConfigTxPower(handle, &txPowerConfig);
    if (RAIL_STATUS_NO_ERROR != rs)
    {
        err1("cfg pwr");
        return rs;
        // Error: The PA could not be initialized due to an improper configuration.
        // Please ensure your configuration is valid for the selected part.
    }

    #if defined(DEFAULT_RFPOWER_RAW) // This will override any other config
        rs = RAIL_SetTxPower(handle, DEFAULT_RFPOWER_RAW);
    #else
        rs = RAIL_SetTxPowerDbm(handle, m_rail_tx_power);
    #endif

    if (RAIL_STATUS_NO_ERROR != rs)
    {
        err1("set pwr");
        return rs;
    }

    // Initialize Radio Calibrations
    rs = RAIL_ConfigCal(handle, RAIL_CAL_ALL);
    if (RAIL_STATUS_NO_ERROR != rs)
    {
        err1("cfg cal");
        return rs;
    }

    // Load custom channel configuration for the generated radio settings
    #ifdef RAIL_USE_CUSTOM_CONFIG
        RAIL_ConfigChannels(handle, channelConfigs[0], &radio_rail_config_changed_cb);
    #else
        rs = RAIL_ConfigChannels(handle, NULL, &radio_rail_config_changed_cb);
        if (0 != rs)
        {
            err1("cfg chan");
            return rs;
        }
    #endif//RAIL_USE_CUSTOM_CONFIG

    RAIL_Events_t events = RAIL_EVENT_CAL_NEEDED
                         | RAIL_EVENT_RX_ACK_TIMEOUT
                         | RAIL_EVENTS_TX_COMPLETION
                         | RAIL_EVENT_RX_PACKET_RECEIVED | RAIL_EVENT_RX_FIFO_OVERFLOW
                         | RAIL_EVENT_TXACK_PACKET_SENT
                         ;
    //                   | RAIL_EVENTS_RX_COMPLETION

    rs = RAIL_ConfigEvents(handle, RAIL_EVENTS_ALL, events);
    if (RAIL_STATUS_NO_ERROR != rs)
    {
        err1("cfg evts");
        return rs;
    }
    // RAIL_ConfigEvents(handle, RAIL_EVENTS_ALL, RAIL_EVENTS_ALL);

    static RAIL_DataConfig_t data_config = {
        .txSource = TX_PACKET_DATA,
        .rxSource = RX_PACKET_DATA,
        .txMethod = PACKET_MODE,
        .rxMethod = PACKET_MODE,
    };
    rs = RAIL_ConfigData(handle, &data_config);
    if (RAIL_STATUS_NO_ERROR != rs)
    {
        err1("cfg dta");
        return rs;
    }

    #ifdef _SILICON_LABS_32B_SERIES_2
        //   - RF2G2_IO1: 0
        //   - RF2G2_IO2: 1
        static RAIL_AntennaConfig_t antennaConfig = { false }; // Zero out structure
        #ifdef DEFAULT_ANTENNA_PATH_IO2
            antennaConfig.defaultPath = 1;
        #else
            antennaConfig.defaultPath = 0;
        #endif
        debug1("cfg ant %d", (int)antennaConfig.defaultPath);
        rs = RAIL_ConfigAntenna(handle, &antennaConfig);
        if (RAIL_STATUS_NO_ERROR != rs)
        {
            err1("cfg ant");
            return rs;
        }
        debug4("cfg ant");
    #endif//_SILICON_LABS_32B_SERIES_2

    rs = RAIL_IEEE802154_Init(handle, &m_radio_ieee802154_config);
    if (RAIL_STATUS_NO_ERROR != rs)
    {
        err1("init");
        return rs;
    }
    debug4("154 init");

    #ifndef RAIL_USE_CUSTOM_CONFIG
        rs = RAIL_IEEE802154_Config2p4GHzRadio(handle);
        if (RAIL_STATUS_NO_ERROR != rs)
        {
            err1("cfg %d", (int)rs);
            return rs;
        }
        debug4("154 cfgd");
    #endif//RAIL_USE_CUSTOM_CONFIG

    // Config2p4GHzRadio triggers invalid action assert when called with some modules and SDK versions
    if (g_rail_invalid_actions > 0)
    {
        err1("invalid:%d", g_rail_invalid_actions);
        return RAIL_STATUS_INVALID_PARAMETER;
    }

    RAIL_IEEE802154_SetPanId(handle, m_radio_pan_id, 0);
    RAIL_IEEE802154_SetShortAddress(handle, m_radio_address, 0);

    RAIL_Idle(handle, RAIL_IDLE, 1);

    debug4("railstartup fifo:%d", (int)m_rx_fifo_status);
    info1("rail txpwr: %d ddBm %d raw", (int)RAIL_GetTxPowerDbm(handle), (int)RAIL_GetTxPower(handle));

    return rs;
}

// RAIL RX FIFO setup ----------------------------------------------------------
#define RAIL_RX_FIFO_SIZE 2048
static uint8_t rail_rx_fifo[RAIL_RX_FIFO_SIZE];
RAIL_Status_t RAILCb_SetupRxFifo(RAIL_Handle_t railHandle)
{
    uint16_t rxFifoSize = RAIL_RX_FIFO_SIZE;
    RAIL_Status_t status = RAIL_SetRxFifo(railHandle, &rail_rx_fifo[0], &rxFifoSize);
    m_rx_fifo_status = status;
    if (rxFifoSize != RAIL_RX_FIFO_SIZE)
    {
        // We set up an incorrect FIFO size
        m_rx_fifo_status = RAIL_STATUS_INVALID_PARAMETER;
        return RAIL_STATUS_INVALID_PARAMETER;
    }
    if (status == RAIL_STATUS_INVALID_STATE)
    {
        // Allow failures due to multiprotocol
        return RAIL_STATUS_NO_ERROR;
    }
    return status;
}
//------------------------------------------------------------------------------

uint32_t radio_sleep_time()
{
    return m_sleep_time;
}

uint32_t radio_tx_packets()
{
    return m_transmitted_packets;
}

uint32_t radio_tx_bytes()
{
    return m_transmitted_bytes;
}

void radio_idle()
{
    sys_panic("basic func"); // radio_basic API function called on RTOS radio
}


void radio_reenable()
{
    sys_panic("basic func"); // radio_basic API function called on RTOS radio
}


static uint32_t radio_timestamp ()
{
    return osKernelGetTickCount();
}


static void radio_send_timeout_cb (void * argument)
{
    osThreadFlagsSet(m_radio_thread_id, RDFLG_RADIO_SEND_TIMEOUT);
}


static comms_error_t radio_start (comms_layer_iface_t * iface, comms_status_change_f * start_done, void * user)
{
    comms_error_t err = COMMS_SUCCESS;
    if (iface != (comms_layer_iface_t *)&m_radio_iface)
    {
        return COMMS_EINVAL;
    }

    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));

    if (ST_RUNNING == m_state)
    {
        err = COMMS_ALREADY;
    }
    else if (ST_OFF != m_state)
    {
        err = COMMS_EBUSY;
    }
    else
    {
        m_state = ST_STARTING;
        m_state_change_cb = start_done;
        m_state_change_user = user;
        osThreadFlagsSet(m_radio_thread_id, RDFLG_RADIO_START);
    }

    osMutexRelease(m_radio_mutex);

    return err;
}


static comms_error_t radio_stop (comms_layer_iface_t* iface, comms_status_change_f * stop_done, void * user)
{
    comms_error_t err = COMMS_SUCCESS;
    if (iface != (comms_layer_iface_t *)&m_radio_iface)
    {
        return COMMS_EINVAL;
    }

    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));

    if (ST_OFF == m_state)
    {
        err = COMMS_ALREADY;
    }
    else if (ST_RUNNING != m_state)
    {
        err = COMMS_EBUSY;
    }
    else
    {
        m_state = ST_STOPPING;
        m_state_change_cb = stop_done;
        m_state_change_user = user;
        osThreadFlagsSet(m_radio_thread_id, RDFLG_RADIO_STOP);
    }

    osMutexRelease(m_radio_mutex);

    return err;
}


static comms_error_t radio_send (comms_layer_iface_t * iface, comms_msg_t * msg,
                                 comms_send_done_f * send_done, void * user)
{
    comms_error_t err = COMMS_FAIL;

    if (iface != (comms_layer_iface_t *)&m_radio_iface)
    {
        return(COMMS_EINVAL);
    }

    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));

    if (ST_RUNNING != m_state)
    {
        err1("radio off");
        err = COMMS_EOFF;
    }
    else if (radio_msg_queue_free != NULL)
    {
        radio_queue_element_t* qm = (radio_queue_element_t*)radio_msg_queue_free;
        radio_msg_queue_free = radio_msg_queue_free->next;

        qm->msg = msg;
        qm->send_done = send_done;
        qm->user = user;
        qm->timestamp_queued = radio_timestamp();
        qm->next = NULL;

        if (radio_msg_queue_head == NULL)
        {
            radio_msg_queue_head = qm;
        }
        else
        {
            radio_queue_element_t* qn = (radio_queue_element_t*)radio_msg_queue_head;
            while (qn->next != NULL)
            {
                qn = qn->next;
            }
            qn->next = qm;
        }

        m_radio_busy = true; // If it wasn't busy before, it is now
        osThreadFlagsSet(m_radio_thread_id, RDFLG_RADIO_SEND);

        info3("snd %p", msg);
        err = COMMS_SUCCESS;
    }
    else
    {
        warn1("busy");
        err = COMMS_ENOMEM;
    }

    osMutexRelease(m_radio_mutex);

    return err;
}


static void radio_send_message (comms_msg_t * msg)
{
    static uint8_t buffer[160];

    if (NULL == msg)
    {
        sys_panic("snull");
    }

    comms_layer_t* iface = (comms_layer_t *)&m_radio_iface;
    RAIL_Status_t rslt;
    uint16_t count, total;
    uint16_t src, dst;
    uint8_t amid;
    RAIL_CsmaConfig_t csmaConf = {0, 0, 1, -75, 320, 128, 0};

    count = comms_get_payload_length(iface, msg);
    src = comms_am_get_source(iface, msg);
    if (src == 0)
    {
        src = m_radio_address;
    }
    dst = comms_am_get_destination(iface, msg);
    if (dst == 0)
    {
        warn1("dest not set");
    }
    amid = comms_get_packet_type(iface, msg);
    // is ack and not broadcast
    if (comms_is_ack_required(iface, msg) && (dst != 0xFFFF))
    {
        radio_tx_wait_ack = true;
        buffer[1] = 0x61;
    }
    else
    {
        radio_tx_wait_ack = false;
        buffer[1] = 0x41;
    }
    buffer[2] = 0x88;
    buffer[3] = m_radio_tx_num;
    buffer[4] = ((m_radio_pan_id >> 0) & (0xFF));
    buffer[5] = ((m_radio_pan_id >> 8) & (0xFF));
    buffer[6] = ((dst >> 0) & 0xFF);
    buffer[7] = ((dst >> 8) & 0xFF);
    buffer[8] = ((src >> 0) & 0xFF);
    buffer[9] = ((src >> 8) & 0xFF);
    buffer[10] = 0x3F;
    // AMID handled below
    memcpy(&buffer[12], comms_get_payload(iface, msg, count), count);

    // Pick correct AMID, add timestamp footer when needed
    if (comms_event_time_valid(iface, msg))
    {
        uint32_t evt_time, diff;
        //debug1("evt time valid");
        buffer[11] = 0x3d; // 3D is used by TinyOS AM for timesync messages

        evt_time = comms_get_event_time(iface, msg);
        diff = evt_time - (radio_timestamp()+1); // It will take at least 448us to get the packet going, round it up

        buffer[12+count] = amid; // Actual AMID is carried after payload
        buffer[13+count] = diff>>24;
        buffer[14+count] = diff>>16;
        buffer[15+count] = diff>>8;
        buffer[16+count] = diff;
        count += 5;
    }
    else
    {
        //debug1("evt time NOT valid");
        buffer[11] = amid;
    }

    buffer[0] = 11 + count + 2; // hdr, data, crc
    total = 1 + 11 + count + 2; // lenb, hdr, data, crc
    //RAIL_WriteTxFifo(m_rail_handle, buffer, count, true);
    RAIL_SetTxFifo(m_rail_handle, buffer, total, sizeof(buffer));

    m_radio_send_timestamp = radio_timestamp();
    m_rail_send_timestamp = m_rail_sent_timestamp = RAIL_GetTime();

    // if ack is required in FCF
    if (radio_tx_wait_ack)
    {
        rslt = RAIL_StartCcaCsmaTx(m_rail_handle, m_radio_channel_current, RAIL_TX_OPTION_WAIT_FOR_ACK, &csmaConf, NULL);
    }
    else
    {
        rslt = RAIL_StartCcaCsmaTx(m_rail_handle, m_radio_channel_current, 0, &csmaConf, NULL);
    }
    debug1("snd %04"PRIX16"->%04"PRIX16"[%02"PRIX8"](%"PRIx8":%"PRIu8")=%d %p l:%d",
           src, dst, amid, m_radio_tx_num, m_csma_retries, rslt, msg, total);

    if (rslt == RAIL_STATUS_NO_ERROR)
    {
        osTimerStart(m_send_timeout_timer, RADIO_MAX_SEND_TIME_MS);
    }
    else
    {
        RAIL_Idle(m_rail_handle, RAIL_IDLE_FORCE_SHUTDOWN, 1);
        RAIL_StartRx(m_rail_handle, m_radio_channel_current, NULL);
        osThreadFlagsSet(m_radio_thread_id, RDFLG_RADIO_SEND_FAIL);
    }
}


static void radio_resend_timeout_cb(void * argument)
{
    osThreadFlagsSet(m_radio_thread_id, RDFLG_RADIO_RESEND);
}


static void radio_send_next()
{
    comms_msg_t * msg = NULL;
    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));
    if (NULL != radio_msg_queue_head)
    {
        radio_msg_sending = radio_msg_queue_head;
        radio_msg_queue_head = radio_msg_queue_head->next;
        m_csma_retries = 0;
        m_radio_tx_num++;

        msg = radio_msg_sending->msg;
    }
    osMutexRelease(m_radio_mutex);

    if (NULL != msg)
    {
        radio_send_message(msg);
    }
}


static void radio_resend()
{
    comms_msg_t * msg = NULL;

    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));
    uint8_t retu = comms_get_retries_used((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg) + 1;
    comms_set_retries_used((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg, retu);
    msg = radio_msg_sending->msg;
    osMutexRelease(m_radio_mutex);

    if (NULL != msg)
    {
        radio_send_message(msg);
    }
}


static void signal_send_done (comms_error_t err)
{
    comms_send_done_f * send_done = NULL;
    comms_msg_t * msgp;
    void * user;
    uint32_t qtime;

    osTimerStop(m_send_timeout_timer);

    assert(NULL != radio_msg_sending);

    user = radio_msg_sending->user;
    msgp = radio_msg_sending->msg;
    send_done = radio_msg_sending->send_done;
    qtime = radio_timestamp() - radio_msg_sending->timestamp_queued;

    radio_msg_sending->next = (radio_queue_element_t*)radio_msg_queue_free;
    radio_msg_queue_free = radio_msg_sending;
    radio_msg_sending = NULL;

    if (err == COMMS_SUCCESS)
    {
        comms_set_timestamp((comms_layer_t *)&m_radio_iface, msgp, radio_timestamp());
        _comms_set_ack_received((comms_layer_t *)&m_radio_iface, msgp);
    }

    if (qtime > 20)
    {
        warn3("slow tx %"PRIu32, qtime);
    }

    logger(err==COMMS_SUCCESS?LOG_INFO3:LOG_WARN3,
          "snt %p e:%d t:(%"PRIu32")(%"PRIu32")",
           msgp, err,
           qtime,
           m_rail_sent_timestamp - m_rail_send_timestamp);

    assert(NULL != send_done);
    send_done((comms_layer_t *)&m_radio_iface, msgp, err, user);
}


static void assertPacketInfo (RAIL_RxPacketInfo_t * packetInfo)
{
    if ((packetInfo->packetBytes > 11) && (packetInfo->firstPortionData == NULL))
    {
        sys_panic("packet");
    }
    if ((packetInfo->packetBytes - packetInfo->firstPortionBytes != 0) && (packetInfo->lastPortionData == NULL))
    {
        sys_panic("packet");
    }
    if (packetInfo->firstPortionBytes > 255)
    {
        sys_panic("packet");
    }
    if (packetInfo->packetBytes > 255)
    {
        sys_panic("packet");
    }
    if (packetInfo->firstPortionBytes > packetInfo->packetBytes)
    {
        sys_panic("packet");
    }
}


static void handle_radio_rx()
{
    // RX processing -----------------------------------------------------------
    RAIL_RxPacketHandle_t rxh;
    if (osOK == osMessageQueueGet(m_rx_queue, &rxh, NULL, 0))
    {
        RAIL_RxPacketInfo_t packetInfo = {0};
        RAIL_RxPacketHandle_t packetHandle = RAIL_GetRxPacketInfo(m_rail_handle, rxh, &packetInfo);
        if (packetHandle != RAIL_RX_PACKET_HANDLE_INVALID)
        {
            RAIL_RxPacketDetails_t packetDetails = {0};
            RAIL_Status_t rx_status = RAIL_GetRxPacketDetailsAlt(m_rail_handle, packetHandle, &packetDetails);
            if (rx_status == RAIL_STATUS_NO_ERROR)
            {
                uint8_t buffer[256] = {0};
                RAIL_RxPacketDetails_t timeDetails = packetDetails;
                bool rts_valid = timeDetails.timeReceived.timePosition != RAIL_PACKET_TIME_INVALID;
                uint32_t rts = 0; // timeReceived.packetTime

                assertPacketInfo(&packetInfo); // Sanity checks

                if (rts_valid)
                {
                    // Account for CRC ... unless someone somewhere configures RAIL_RX_OPTION_STORE_CRC?
                    timeDetails.timeReceived.totalPacketBytes = packetInfo.packetBytes + 2; // + CRC_BYTES;
                    // Want the earliest timestamp possible
                    rx_status = RAIL_GetRxTimePreambleStartAlt(m_rail_handle, &timeDetails);
                    if (rx_status == RAIL_STATUS_NO_ERROR)
                    {
                        rts = timeDetails.timeReceived.packetTime;
                    }
                    else
                    {
                        rts_valid = false;
                    }
                }

                if (rts_valid == false)
                {
                    warn1("rts %d %d", packetDetails.timeReceived.timePosition, timeDetails.timeReceived.timePosition);
                }

                RAIL_CopyRxPacket(buffer, &packetInfo);

                RAIL_Status_t rst = RAIL_ReleaseRxPacket(m_rail_handle, packetHandle);
                if (rst != RAIL_STATUS_NO_ERROR)
                {
                    warnb1("rst", &rst, sizeof(RAIL_Status_t));
                    sys_panic("release");
                }

                uint16_t currTime = (uint16_t)(radio_timestamp() >> 10);
                uint16_t source = ((uint16_t)buffer[8] << 0) | ((uint16_t)buffer[9] << 8);

                if ((!radio_seqNum_save(source, buffer[3], currTime)) && (packetInfo.packetBytes >= 12))
                {
                    warn3("same seqNum:%02"PRIX8, buffer[3]);
                }
                else if ((packetInfo.packetBytes >= 12)
                       &&(buffer[2] == 0x88)&&(buffer[5] == 0x00) && (buffer[10] == 0x3F))
                {
                    comms_msg_t msg;
                    am_id_t amid;
                    void* payload;
                    uint8_t plen;
                    uint8_t lqi = 0xFF;
                    uint32_t timestamp = radio_timestamp() - (RAIL_GetTime() - rts)/1000;

                    comms_init_message((comms_layer_t *)&m_radio_iface, &msg);
                    if (buffer[11] == 0x3D)
                    {
                        int32_t diff = (buffer[packetInfo.packetBytes - 4] << 24) |
                                   (buffer[packetInfo.packetBytes - 3] << 16) |
                                   (buffer[packetInfo.packetBytes - 2] << 8) |
                                   (buffer[packetInfo.packetBytes - 1]);

                        if ((packetInfo.packetBytes < 17)
                          ||(packetInfo.packetBytes > 200))
                        {
                            sys_panic("packet");
                        }
                        amid = buffer[(packetInfo.packetBytes-5)];
                        plen = packetInfo.packetBytes - 17;
                        if (rts_valid)
                        {
                            comms_set_event_time((comms_layer_t *)&m_radio_iface, &msg, (uint32_t)(diff + timestamp));
                        }
                    }
                    else
                    {
                        amid = buffer[11];
                        plen = packetInfo.packetBytes - 12;
                    }

                    payload = comms_get_payload((comms_layer_t *)&m_radio_iface, &msg, plen);

                    if (NULL != payload)
                    {
                        uint16_t dest = ((uint16_t)buffer[6] << 0) | ((uint16_t)buffer[7] << 8);

                        comms_set_packet_type((comms_layer_t *)&m_radio_iface, &msg, amid);
                        comms_set_payload_length((comms_layer_t *)&m_radio_iface, &msg, plen);
                        memcpy(payload, (const void *)&buffer[12], plen);

                        if (rts_valid)
                        {
                            comms_set_timestamp((comms_layer_t *)&m_radio_iface, &msg, timestamp);
                        }
                        _comms_set_rssi((comms_layer_t *)&m_radio_iface, &msg, packetDetails.rssi);
                        if (packetDetails.rssi < -96)
                        {
                            lqi = 0;
                        }
                        else if (packetDetails.rssi < -93) // RFR2-like LQI simulation
                        {
                            lqi = lqi + (packetDetails.rssi+93)*50;
                        }
                        _comms_set_lqi((comms_layer_t *)&m_radio_iface, &msg, lqi);
                        comms_am_set_destination((comms_layer_t *)&m_radio_iface, &msg, dest);
                        comms_am_set_source((comms_layer_t *)&m_radio_iface, &msg, source);

                        debugb1("rx %04"PRIX16"->%04"PRIX16"[%02"PRIX8"] %"PRIu32" r:%"PRIi8" l:%"PRIu8" %"PRIu8":",
                                &(buffer[12]), 8,
                                source, dest, amid,
                                rts,
                                packetDetails.rssi, lqi, plen);

                        debug2("Rx[%02"PRIX8"] %04"PRIX16"->%04"PRIX16" rssi:%"PRIi8"",
                                buffer[12],
                                source, dest,
                                packetDetails.rssi);

                        comms_deliver((comms_layer_t *)&m_radio_iface, &msg);
                    }
                    else warn1("rx bad pl %02"PRIX8" %"PRIu8, amid, plen);
                }
                else warnb1("rx bad l=%"PRIu16, buffer, packetInfo.packetBytes > 128 ? 128: packetInfo.packetBytes, packetInfo.packetBytes);
            }
            else err1("rxd");
        }
        else err1("rxi");

        // There might be more packets in the queue, but don't let RX swamp the
        // radio - defer it to the next run through the loop
        osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_RX_MORE);
    }
}


static void update_tx_stats(comms_msg_t * msg)
{
    m_transmitted_packets++; // Packet was actually sent out
    m_transmitted_bytes += (14 // 12 bytes header + 2 bytes CRC
        + comms_get_payload_length((comms_layer_t *)&m_radio_iface, msg)
        + (comms_event_time_valid((comms_layer_t *)&m_radio_iface, msg) ? 5 : 0)
        + 2);
}


static void handle_radio_tx (uint32_t flags)
{
    // If sending, see if it has completed
    if (NULL != radio_msg_sending)
    {
        // Sending has completed -----------------------------------------------
        if (flags & RDFLG_RAIL_SEND_DONE)
        {
            update_tx_stats(radio_msg_sending->msg);

            if (radio_tx_wait_ack) // Alternatively we should get rx_ack_timeout
            {
                debug1("ackd");
                comms_ack_received((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg);
            }
            signal_send_done(COMMS_SUCCESS);
        }
        // Ack was not received ------------------------------------------------
        else if (flags & RDFLG_RAIL_RXACK_TIMEOUT)
        {
            bool resend = false;

            osTimerStop(m_send_timeout_timer);

            update_tx_stats(radio_msg_sending->msg);

            if (comms_get_retries_used((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg)
             < comms_get_retries((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg))
            {
                resend = true;
            }

            logger(resend?LOG_DEBUG1:LOG_WARN3, "rx ackTimeout (%"PRIu8"/%"PRIu8")",
                   comms_get_retries_used((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg),
                   comms_get_retries((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg));
            if (resend)
            {
                m_csma_retries = 0;
                osTimerStart(m_resend_timer,
                             comms_get_timeout((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg));
            }
            else
            {
                signal_send_done(COMMS_ENOACK);
            }
        }
        // CSMA has failed to transmit the message -----------------------------
        else if (flags & RDFLG_RAIL_SEND_BUSY)
        {
            bool resend = false;

            osTimerStop(m_send_timeout_timer);

            if (m_csma_retries < 7)
            {
                resend = true;
                m_csma_retries++;
            }

            if (resend)
            {
                radio_send_message(radio_msg_sending->msg);
            }
            else
            {
                if (comms_get_retries_used((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg)
                 < comms_get_retries((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg))
                {
                    m_csma_retries = 0;
                    osTimerStart(m_resend_timer, comms_get_timeout((comms_layer_t *)&m_radio_iface, radio_msg_sending->msg));
                }
                else
                {
                    signal_send_done(COMMS_EBUSY);
                }
            }
        }
        // Sending has failed in some generic way ------------------------------
        else if (flags & RDFLG_RADIO_SEND_FAIL)
        {
            signal_send_done(COMMS_FAIL);
        }
        // Sending has not completed in a reasonable amount of time ------------
        else if (flags & RDFLG_RADIO_SEND_TIMEOUT)
        {
            // Check that an actual timeout has happened and this is not race
            uint32_t passed = radio_timestamp() - m_radio_send_timestamp;
            if (passed >= RADIO_MAX_SEND_TIME_MS)
            {
                err1("TIMEOUT");
                signal_send_done(COMMS_ETIMEOUT);

                // Presumably something is wrong with the radio
                osThreadFlagsSet(m_radio_thread_id, RDFLG_RADIO_RESTART);
            }
            else // Perhaps triggered because timer from previous send was not stopped in time
            {
                warn1("timeout %"PRIu32, passed);
                osTimerStart(m_send_timeout_timer, RADIO_MAX_SEND_TIME_MS - passed);
            }
        }
        // PacketLink resend functionality -------------------------------------
        else if (flags & RDFLG_RADIO_RESEND)
        {
            radio_resend();
        }
    }
}


static void handle_radio_events ()
{
    // RX busy handling -----------------------------------------------------
    if (rx_busy || rx_overflow)
    {
        uint8_t rxb __attribute__((unused));
        uint8_t rxo __attribute__((unused));
        CORE_irqState_t irqState = CORE_EnterCritical();
        rxb = rx_busy;
        rxo = rx_overflow;
        rx_busy = 0;
        rx_overflow = 0;
        CORE_ExitCritical(irqState);
        warn1("rx b:%"PRIu8" o:%"PRIu8, rxb, rxo);
    }

    // RX failure handling -----------------------------------------------------
    if ((rx_abort > 100) || rx_fail)
    {
        uint8_t rxa __attribute__((unused));
        uint8_t rxf __attribute__((unused));
        CORE_irqState_t irqState = CORE_EnterCritical();
        rxa = rx_abort;
        rxf = rx_fail;
        rx_abort = 0;
        rx_fail = 0;
        CORE_ExitCritical(irqState);
        warn1("rx a:%"PRIu8" f:%"PRIu8, rxa, rxf);
    }

    // RX frame error handling -----------------------------------------------------
    if (rx_frame_error)
    {
        uint8_t rxfe __attribute__((unused));
        CORE_irqState_t irqState = CORE_EnterCritical();
        rxfe = rx_frame_error;
        rx_frame_error = 0;
        CORE_ExitCritical(irqState);
        warn1("rx fe:%"PRIu8, rxfe);
    }

    // TX ack sent -----------------------------------------------------------------
    if (tx_ack_sent)
    {
        uint8_t tas __attribute__((unused));
        CORE_irqState_t irqState = CORE_EnterCritical();
        tas = tx_ack_sent;
        tx_ack_sent = 0;
        CORE_ExitCritical(irqState);
        info4("tx_ack_sent:%"PRIu8, tas);
    }
}


static void start_radio_now ()
{
    info2("start");

    SLEEP_SleepBlockBegin(sleepEM2);
    m_sleep_time += radio_timestamp() - m_stop_timestamp;

    if(RAIL_STATUS_NO_ERROR != radio_rail_configure(m_rail_handle))
    {
        err1("rail cfg");
        osDelay(1000);
        sys_panic("rail");
    }

    RAIL_Idle(m_rail_handle, RAIL_IDLE, 1);

    m_radio_channel_current = m_radio_channel_configured;

    RAIL_Status_t s;
    if (m_stream_mode_enabled)
    {
        m_stream_mode_active = true;
        s = RAIL_StartTxStream(m_rail_handle, m_radio_channel_current, m_stream_mode);
    }
    else
    {
        s = RAIL_StartRx(m_rail_handle, m_radio_channel_current, NULL);
    }
    if (s != RAIL_STATUS_NO_ERROR)
    {
        err1("rail err: %"PRIu8"", s);
    }

    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));
    m_state = ST_RUNNING;
    osMutexRelease(m_radio_mutex);

    m_state_change_cb((comms_layer_t *)&m_radio_iface, COMMS_STARTED, m_state_change_user);
}


static uint32_t rail_packet_age(RAIL_RxPacketHandle_t handle)
{
    RAIL_RxPacketInfo_t packetInfo = {0};
    RAIL_RxPacketHandle_t ph = RAIL_GetRxPacketInfo(m_rail_handle, handle, &packetInfo);
    if (ph != RAIL_RX_PACKET_HANDLE_INVALID)
    {
        RAIL_RxPacketDetails_t packetDetails = {0};
        RAIL_Status_t status = RAIL_GetRxPacketDetailsAlt(m_rail_handle, ph, &packetDetails);
        if (status == RAIL_STATUS_NO_ERROR)
        {
            RAIL_RxPacketDetails_t timeDetails = packetDetails;
            if (RAIL_PACKET_TIME_INVALID != timeDetails.timeReceived.timePosition)
            {
                // Account for CRC ... unless someone somewhere configures RAIL_RX_OPTION_STORE_CRC?
                timeDetails.timeReceived.totalPacketBytes = packetInfo.packetBytes + 2; // + CRC_BYTES;
                // Want the earliest timestamp possible
                if (RAIL_STATUS_NO_ERROR == RAIL_GetRxTimePreambleStartAlt(m_rail_handle, &timeDetails))
                {
                    return RAIL_GetTime() - timeDetails.timeReceived.packetTime;
                }
            }
        }
    }
    return UINT32_MAX;
}


static void stop_radio_now ()
{
    info2("stop");

    if (m_stream_mode_active)
    {
        RAIL_StopTxStream(m_rail_handle);
        m_stream_mode_active = false;
    }

    RAIL_Idle(m_rail_handle, RAIL_IDLE, 1);

    // Return any pending TX messages with COMMS_EOFF
    // No mutex, queue cannot change - send not accepting msgs in stop state
    while (NULL != radio_msg_queue_head)
    {
        radio_queue_element_t* qe = radio_msg_queue_head;
        warn1("rm txmsg %p", qe->msg);
        qe->send_done((comms_layer_t *)&m_radio_iface, qe->msg, COMMS_EOFF, qe->user);

        radio_msg_queue_head = qe->next;
        qe->next = radio_msg_queue_free;
        radio_msg_queue_free = qe;
    }

    // Discard any pending RX messages
    RAIL_RxPacketHandle_t rxh;
    while (osOK == osMessageQueueGet(m_rx_queue, &rxh, NULL, 0))
    {
        if (rail_packet_age(rxh) > 5000)
        {
            warn2("rm rxmsg age:%"PRIu32"us", rail_packet_age(rxh));
        }
        RAIL_Status_t rst = RAIL_ReleaseRxPacket(m_rail_handle, rxh);
        if (rst != RAIL_STATUS_NO_ERROR)
        {
            warnb1("rst", &rst, sizeof(RAIL_Status_t));
            sys_panic("release");
        }
    }

    m_stop_timestamp = radio_timestamp();
    SLEEP_SleepBlockEnd(sleepEM2);

    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));
    m_state = ST_OFF;
    osMutexRelease(m_radio_mutex);

    m_state_change_cb((comms_layer_t *)&m_radio_iface, COMMS_STOPPED, m_state_change_user);
}


static void radio_thread (void * p)
{
    bool running = false;

    m_rail_handle = radio_rail_init();
    if (NULL == m_rail_handle)
    {
        err1("radio init");
        osDelay(1000);
        sys_panic("rail");
    }

    for (;;)
    {
        uint32_t flags = osFlagsErrorTimeout;
        RadioState_t state;

        while(osFlagsErrorTimeout == flags)
        {
            uint32_t wait = osWaitForever;
            if (running) // must prevent sleep, but permit thread switches if no flags
            {
                #if defined(configUSE_TICKLESS_IDLE) && (configUSE_TICKLESS_IDLE == 1)
                    #if configEXPECTED_IDLE_TIME_BEFORE_SLEEP > 1
                        wait = configEXPECTED_IDLE_TIME_BEFORE_SLEEP - 1;
                    #else
                        wait = 1; // 1 should be enough to prevent sleep
                    #endif
                #endif
            }
            flags = osThreadFlagsWait(RDFLGS_ALL, osFlagsWaitAny, wait);
        }

        while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));
        state = m_state;
        m_radio_busy = (radio_msg_sending != NULL)||(radio_msg_queue_head != NULL);
        osMutexRelease(m_radio_mutex);

        if (ST_STARTING == state)
        {
            start_radio_now();
            running = true;
        }

        // If an exception has occurred and RAIL is broken ---------------------
        if (flags & RDFLG_RADIO_RESTART)
        {
            warn1("restart");
            m_rail_handle = radio_rail_init();
            if (m_rail_handle == NULL)
            {
                sys_panic("rail");
            }

            // If sending, cancel and notify user
            if (NULL != radio_msg_sending)
            {
                flags |= RDFLG_RADIO_SEND_FAIL;
            }
        }

        // Handle TX activities
        handle_radio_tx(flags);

        // Check RX queue and process any messages there
        handle_radio_rx(flags);

        // Handle "other" events
        handle_radio_events();

        if (radio_msg_sending == NULL)
        {
            if (ST_STOPPING == state)
            {
                stop_radio_now(); // Will return queued messages
                running = false;
            }
            else
            {
                radio_send_next(); // Won't do anything if nothing queued
            }
        }

        if (flags & RDFLG_RADIO_DEINIT)
        {
            if (state == ST_OFF)
            {
                debug1("deinit");
                osThreadExit();
            }
        }
    }
}


bool radio_poll ()
{
    bool busy;

    while (osOK != osMutexAcquire(m_radio_mutex, osWaitForever));
    busy = m_radio_busy;
    osMutexRelease(m_radio_mutex);

    return busy;
}


static void radio_rail_event_cb (RAIL_Handle_t m_rail_handle, RAIL_Events_t events)
{
    if (events & RAIL_EVENTS_TX_COMPLETION)
    {
        if (events & RAIL_EVENT_TX_PACKET_SENT)
        {
            m_rail_sent_timestamp = RAIL_GetTime();
            if (radio_tx_wait_ack)
            {
                // Wait for the ack or the RAIL_EVENT_RX_ACK_TIMEOUT event
            }
            else
            {
                osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_SEND_DONE);
            }
        }
        else
        {
            if (events & RAIL_EVENT_TX_CHANNEL_BUSY)
            {
                osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_SEND_BUSY);
            }
            else // (RAIL_EVENT_TX_BLOCKED | RAIL_EVENT_TX_ABORTED | RAIL_EVENT_TX_UNDERFLOW)
            {
                osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_SEND_FAIL);
            }
        }
    }

    if (events & RAIL_EVENTS_RX_COMPLETION)
    {
        bool unhandled = true;
        if (events & RAIL_EVENT_RX_PACKET_RECEIVED)
        {
            RAIL_RxPacketHandle_t rxh = RAIL_HoldRxPacket(m_rail_handle);
            if (rxh != RAIL_RX_PACKET_HANDLE_INVALID)
            {
                RAIL_RxPacketInfo_t pi;
                if (RAIL_GetRxPacketInfo(m_rail_handle, rxh, &pi) == rxh)
                {
                    // Inspect if it is an ack
                    if (pi.packetBytes == 4)
                    {
                        uint8_t buffer[4];
                        RAIL_CopyRxPacket(buffer, &pi);
                        if ((buffer[0] == 0x05) && (buffer[1] == 0x02) && (buffer[3] == m_radio_tx_num))
                        {
                            if (radio_tx_wait_ack)
                            {
                                osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_SEND_DONE);
                            }
                            RAIL_ReleaseRxPacket(m_rail_handle, rxh);
                            rxh = RAIL_RX_PACKET_HANDLE_INVALID;
                        }
                    }
                }

                // packet would have been discarded if it was ack
                if (rxh != RAIL_RX_PACKET_HANDLE_INVALID)
                {
                    if (osOK != osMessageQueuePut(m_rx_queue, &rxh, 0, 0))
                    {
                        RAIL_ReleaseRxPacket(m_rail_handle, rxh);
                        rx_busy++;
                        osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_RX_BUSY);
                    }
                    else
                    {
                        osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_RX_SUCCESS);
                    }
                }
            }
            else
            {
                rx_busy++;
            }
            unhandled = false;
        }
        if (events & RAIL_EVENT_RX_FIFO_OVERFLOW)
        {
            unhandled = false;
            rx_overflow++;
            osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_RX_OVERFLOW);
        }
        if (events & RAIL_EVENT_RX_ADDRESS_FILTERED)
        {
            unhandled = false; // don't care
        }
        if (events & RAIL_EVENT_RX_FRAME_ERROR)
        {
            unhandled = false;
            rx_frame_error++;
            osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_RX_FRAME_ERROR);
        }
        if (events & RAIL_EVENT_RX_PACKET_ABORTED)
        {
            unhandled = false;
            rx_abort++;
            osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_RX_ABORT);
        }

        if (unhandled)
        {
            rx_fail++;
            osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_RX_FAIL);
        }
    }

    if (events & RAIL_EVENT_TXACK_PACKET_SENT)
    {
        tx_ack_sent++;
        osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_TXACK_SENT);
    }

    if (events & RAIL_EVENT_RX_ACK_TIMEOUT)
    {
        if (radio_tx_wait_ack)
        {
            osThreadFlagsSet(m_radio_thread_id, RDFLG_RAIL_RXACK_TIMEOUT);
        }
    }

    if (events & RAIL_EVENT_CAL_NEEDED)
    {
        //printf("RAIL EVENT CAL NEEDED\n");
        RAIL_Calibrate(m_rail_handle, NULL, RAIL_CAL_ALL_PENDING);
    }
}


RAIL_AssertErrorCodes_t global_rail_error_code;
void RAILCb_AssertFailed (RAIL_Handle_t railHandle, RAIL_AssertErrorCodes_t errorCode)
{
    if (errorCode == RAIL_ASSERT_FAILED_UNEXPECTED_STATE_RX_FIFO)
    {
        RAIL_Idle(railHandle, RAIL_IDLE, true);
        osThreadFlagsSet(m_radio_thread_id, RDFLG_RADIO_RESTART);
    }
    else if (errorCode == RAIL_ASSERT_INVALID_MODULE_ACTION)
    {
        g_rail_invalid_actions++;
    }
    else
    {
        global_rail_error_code = errorCode;
        sys_panic("railerr");
    }
}
