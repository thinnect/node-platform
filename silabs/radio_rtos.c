/*
 * Mist-comm compatible SiLabs RAIL based radio layer for FreeRTOS.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Veiko Rütter, Konstantin Bilozor, Raido Pahtma
 */

#include "radio.h"
#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include "rail.h"
#include "rail_config.h"
#include "rail_ieee802154.h"
#include "rail_types.h"
#include "rail_chip_specific.h"
#include "rail_assert_error_codes.h"
#include "pa_conversions_efr32.h"
#include "pa_curves_efr32.h"

#include "cmsis_os2.h"
// Because including FreeRTOS conflicts with SiLabs
extern void vPortEnterCritical();
extern void vPortExitCritical();

#include "mist_comm_iface.h"
#include "mist_comm_am.h"

#include "radio_seqNum.h"

#include "loglevels.h"
#define __MODUUL__ "radio"
#define __LOG_LEVEL__ (LOG_LEVEL_radio & BASE_LOG_LEVEL)
#include "log.h"

uint16_t radio_address;
static uint16_t radio_pan_id;
static uint16_t radio_channel;
comms_layer_am_t radio_iface;

static osMutexId_t radio_mutex;
static osTimerId_t radio_send_timeout_timer;
static osTimerId_t radio_resend_timer;

static RAIL_Handle_t radio_rail_handle;
static RAIL_Status_t rx_fifo_status;
static uint8_t radio_tx_num;
static bool radio_tx_wait_ack;

static volatile bool radio_send_done_flag;
static volatile bool radio_send_busy;
static volatile bool radio_send_fail;
static volatile bool radio_send_timeout;
static volatile uint8_t rx_busy;
static volatile uint8_t rx_overflow;
static volatile uint8_t rx_frame_error;
static volatile uint8_t rx_abort;
static volatile uint8_t rx_fail;
static volatile bool rx_ack_timeout;
static volatile uint8_t tx_ack_sent;
static volatile bool radio_restart;
static volatile uint8_t newSrcPos;

static uint32_t radio_send_time;
static uint32_t radio_sent_time;
static uint8_t radio_send_retries;

static void radio_thread(void *p);

static RAIL_Handle_t radio_rail_init(); // Internal RAIL initialization procedures
static void radio_rail_event_cb(RAIL_Handle_t radio_rail_handle, RAIL_Events_t events);
static void radio_rail_radio_config_changed_cb(RAIL_Handle_t radio_rail_handle, const RAIL_ChannelConfigEntry_t *entry);
static void radio_rail_rfready_cb(RAIL_Handle_t radio_rail_handle);
static uint32_t radio_timestamp();
static void radio_send_timeout_callback(void* argument);
static void radio_resend_timeout_callback(void* argument);

static comms_error_t radio_send(comms_layer_iface_t *iface, comms_msg_t *msg, comms_send_done_f *send_done, void *user);

static radio_queue_element_t radio_msg_queue_memory[7];
static volatile radio_queue_element_t* radio_msg_queue_free;
static volatile radio_queue_element_t* radio_msg_queue_head;
static volatile radio_queue_element_t* radio_msg_sending;

osMessageQueueId_t rxQueue;

comms_layer_t* radio_init(uint16_t channel, uint16_t pan_id, uint16_t address) {
	radio_channel = channel;
	radio_pan_id = pan_id;
	radio_address = address;
	radio_tx_num = 0;
	newSrcPos = 0;

	radio_msg_sending = NULL;
	radio_msg_queue_head = NULL;
	radio_msg_queue_free = &radio_msg_queue_memory[0];
	radio_msg_queue_free->next = NULL;
	for(uint8_t i=1;i<sizeof(radio_msg_queue_memory)/sizeof(radio_queue_element_t);i++) {
		radio_msg_queue_memory[i].next = (radio_queue_element_t*)radio_msg_queue_free;
		radio_msg_queue_free = &radio_msg_queue_memory[i];
	}

	rxQueue = osMessageQueueNew(10, sizeof(RAIL_RxPacketHandle_t), NULL);
	if(rxQueue == NULL) {
		err1("rxq");
		return(NULL);
	}

	radio_rail_handle = radio_rail_init();
	if(radio_rail_handle != NULL) {
		radio_mutex = osMutexNew(NULL);
		radio_send_timeout_timer = osTimerNew(&radio_send_timeout_callback, osTimerOnce, NULL, NULL);
		radio_resend_timer = osTimerNew(&radio_resend_timeout_callback, osTimerOnce, NULL, NULL);
		const osThreadAttr_t radio_thread_attr = {
			.name = "radio"
		};
		osThreadNew(radio_thread, NULL, &radio_thread_attr);
		comms_am_create((comms_layer_t *)&radio_iface, radio_address, radio_send);
		return (comms_layer_t *)&radio_iface;
	}
	err1("rail_init");
	return(NULL);
}

RAIL_Handle_t radio_rail_init() {
	RAIL_Handle_t handle;

	static RAIL_Config_t rail_config = {
		.eventsCallback = &radio_rail_event_cb
	};
	static RAIL_IEEE802154_Config_t ieee802154_config = {
		.addresses = NULL,
		.ackConfig = {
			.enable = 1,
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
		.promiscuousMode = 0,
		.isPanCoordinator = 0
	};
	static RAIL_DataConfig_t data_config = {
		.txSource = TX_PACKET_DATA,
		.rxSource = RX_PACKET_DATA,
		.txMethod = PACKET_MODE,
		.rxMethod = PACKET_MODE,
	};
	//RAIL_DECLARE_TX_POWER_VBAT_CURVES(piecewiseSegments, curvesSg, curves24Hp, curves24Lp);
	RAIL_DECLARE_TX_POWER_VBAT_CURVES_ALT;

	radio_restart = false;
	radio_send_done_flag = false;
	radio_send_busy = false;
	radio_send_fail = false;
	radio_send_timeout = false;
	rx_abort = 0;
	rx_busy = 0;
	rx_overflow = 0;
	rx_fail = 0;
	rx_fifo_status = RAIL_STATUS_NO_ERROR-1;

	int32_t priority = 3; // not shifted, but once shifted = 01100000
	NVIC_SetPriority(FRC_PRI_IRQn, priority);
	NVIC_SetPriority(FRC_IRQn, priority);
	NVIC_SetPriority(MODEM_IRQn, priority);
	NVIC_SetPriority(RAC_SEQ_IRQn, priority);
	NVIC_SetPriority(RAC_RSM_IRQn, priority);
	NVIC_SetPriority(BUFC_IRQn, priority);
	NVIC_SetPriority(AGC_IRQn, priority);
	NVIC_SetPriority(PROTIMER_IRQn, priority);
	NVIC_SetPriority(SYNTH_IRQn, priority);
	//NVIC_SetPriority(RFSENSE_IRQn, priority); // Not supported on Series2 ?

	handle = RAIL_Init(&rail_config, &radio_rail_rfready_cb);
	if(handle == NULL) {
		// printf("RAIL INIT ERROR\n");
		return(NULL);
	}

	// Put the variables declared above into the appropriate structure
  	//RAIL_TxPowerCurvesConfig_t txPowerCurvesConfig = { curves24Hp, curvesSg, curves24Lp, piecewiseSegments };
  	RAIL_TxPowerCurvesConfigAlt_t txPowerCurvesConfig = RAIL_DECLARE_TX_POWER_CURVES_CONFIG_ALT;

	// In the Silicon Labs implementation, the user is required to save those curves into
	// to be referenced when the conversion functions are called
	//RAIL_InitTxPowerCurves(&txPowerCurvesConfig);
	RAIL_InitTxPowerCurvesAlt(&txPowerCurvesConfig);

	RAIL_EnablePaCal(1);

	// Declare the structure used to configure the PA
	//RAIL_TxPowerConfig_t txPowerConfig = { RAIL_TX_POWER_MODE_2P4_HP, 1800, 10 };
	RAIL_TxPowerConfig_t txPowerConfig = { RAIL_TX_POWER_MODE_2P4_HP, 3300, 10 };

	if(RAIL_ConfigTxPower(handle, &txPowerConfig) != RAIL_STATUS_NO_ERROR) {
		// printf("TX POWER CONF ERROR\n");
		// Error: The PA could not be initialized due to an improper configuration.
		// Please ensure your configuration is valid for the selected part.
		return(NULL);
	}

	RAIL_TxPower_t power = -140; // Testsystem power: -16.8dBm
	RAIL_GetTxPowerConfig(handle, &txPowerConfig);
	RAIL_TxPowerLevel_t powerLevel = RAIL_ConvertDbmToRaw(handle, txPowerConfig.mode, power);

	RAIL_SetTxPower(handle, powerLevel);

	// Initialize Radio Calibrations
	RAIL_ConfigCal(handle, RAIL_CAL_ALL);

	// Load the channel configuration for the generated radio settings
	//RAIL_ConfigChannels(handle, channelConfigs[0], &radio_rail_radio_config_changed_cb);
	(void)radio_rail_radio_config_changed_cb; // disabled, because crashes Series2 startup

	RAIL_Events_t events = RAIL_EVENT_CAL_NEEDED
	                     | RAIL_EVENT_RX_ACK_TIMEOUT
	                     | RAIL_EVENTS_TX_COMPLETION
	                     | RAIL_EVENT_RX_PACKET_RECEIVED | RAIL_EVENT_RX_FIFO_OVERFLOW
	                     | RAIL_EVENT_TXACK_PACKET_SENT
	                     ;
	//           | RAIL_EVENTS_RX_COMPLETION

	RAIL_ConfigEvents(handle, RAIL_EVENTS_ALL, events);
	// RAIL_ConfigEvents(handle, RAIL_EVENTS_ALL, RAIL_EVENTS_ALL);

	RAIL_ConfigData(handle, &data_config);

	RAIL_IEEE802154_Config2p4GHzRadio(handle);
	RAIL_IEEE802154_Init(handle, &ieee802154_config);
	RAIL_IEEE802154_SetPanId(handle, radio_pan_id, 0);
	RAIL_IEEE802154_SetShortAddress(handle, radio_address, 0);

	RAIL_Idle(handle, RAIL_IDLE, 1);
	if(RAIL_StartRx(handle, radio_channel, NULL) != RAIL_STATUS_NO_ERROR) {
		err1("StartRx");
	}
	debug1("railstartup fifo:%d", rx_fifo_status);
	return handle;
}

#define RAIL_RX_FIFO_SIZE 2048
static uint8_t rail_rx_fifo[RAIL_RX_FIFO_SIZE];
RAIL_Status_t RAILCb_SetupRxFifo(RAIL_Handle_t railHandle) {
	uint16_t rxFifoSize = RAIL_RX_FIFO_SIZE;
	RAIL_Status_t status = RAIL_SetRxFifo(railHandle, &rail_rx_fifo[0], &rxFifoSize);
	rx_fifo_status = status;
	if(rxFifoSize != RAIL_RX_FIFO_SIZE) {
		// We set up an incorrect FIFO size
		rx_fifo_status = RAIL_STATUS_INVALID_PARAMETER;
		return RAIL_STATUS_INVALID_PARAMETER;
	}
	if(status == RAIL_STATUS_INVALID_STATE) {
		// Allow failures due to multiprotocol
    	return RAIL_STATUS_NO_ERROR;
	}
	return status;
}

void radio_idle() {
	RAIL_Idle(radio_rail_handle, RAIL_IDLE, 1);
}

void radio_reenable() {
	RAIL_StartRx(radio_rail_handle, radio_channel, NULL);
}

static uint32_t radio_timestamp() {
	return osKernelGetTickCount();
}

static void radio_send_timeout_callback(void* argument) {
	while(osMutexAcquire(radio_mutex, 1000) != osOK);
	if(radio_msg_sending != NULL) {
		radio_send_timeout = true;
	}
	osMutexRelease(radio_mutex);
}

static comms_error_t radio_send(comms_layer_iface_t *iface, comms_msg_t *msg, comms_send_done_f *send_done, void *user) {
	comms_error_t err = COMMS_FAIL;

	if(iface != (comms_layer_iface_t *)&radio_iface) {
		return(COMMS_EINVAL);
	}

	while(osMutexAcquire(radio_mutex, 1000) != osOK);

	if(radio_msg_queue_free != NULL) {
		radio_queue_element_t* qm = (radio_queue_element_t*)radio_msg_queue_free;
		radio_msg_queue_free = radio_msg_queue_free->next;

		qm->msg = msg;
		qm->send_done = send_done;
		qm->user = user;
		qm->next = NULL;

		if(radio_msg_queue_head == NULL) {
			radio_msg_queue_head = qm;
		} else {
			radio_queue_element_t* qn = (radio_queue_element_t*)radio_msg_queue_head;
			while(qn->next != NULL) {
				qn = qn->next;
			}
			qn->next = qm;
		}
		info1("snd %p", msg);
		err = COMMS_SUCCESS;
	} else {
		warn1("busy");
		err = COMMS_ENOMEM;
	}

	osMutexRelease(radio_mutex);

	return err;
}

static void radio_send_message(comms_msg_t* msg) {
	static uint8_t buffer[160];

	while(osMutexAcquire(radio_mutex, 1000) != osOK);

	comms_layer_t* iface = (comms_layer_t *)&radio_iface;
	RAIL_Status_t rslt;
	uint16_t count, total;
	uint16_t src, dst;
	uint8_t amid;
	RAIL_CsmaConfig_t csmaConf = {0, 0, 1, -75, 320, 128, 0};

	count = comms_get_payload_length(iface, msg);
	src = comms_am_get_source(iface, msg);
	if(src == 0) {
		src = radio_address;
	}
	dst = comms_am_get_destination(iface, msg);
	if(dst == 0) {
		warn1("dest not set");
	}
	amid = comms_get_packet_type(iface, msg);
	// is ack and not broadcast
	if(comms_is_ack_required(iface, msg) && (dst != 0xFFFF)) {
		radio_tx_wait_ack = true;
		buffer[1] = 0x61;
	} else {
		radio_tx_wait_ack = false;
		buffer[1] = 0x41;
	}
	buffer[2] = 0x88;
	buffer[3] = radio_tx_num;
	buffer[4] = ((radio_pan_id >> 0) & (0xFF));
	buffer[5] = ((radio_pan_id >> 8) & (0xFF));
	buffer[6] = ((dst >> 0) & 0xFF);
	buffer[7] = ((dst >> 8) & 0xFF);
	buffer[8] = ((src >> 0) & 0xFF);
	buffer[9] = ((src >> 8) & 0xFF);
	buffer[10] = 0x3F;
	buffer[11] = amid;
	if(comms_event_time_valid(iface, msg)) {
		uint32_t evt_time, diff;
		//debug1("evt time valid");
		buffer[11] = 0x3d;

		memcpy(&buffer[12], comms_get_payload(iface, msg, count), count);

		evt_time = comms_get_event_time(iface, msg);
		diff = evt_time - (radio_timestamp()+1); // It will take at least 448us to get the packet going, round it up

		buffer[12+count] = amid;
		buffer[13+count] = diff>>24;
		buffer[14+count] = diff>>16;
		buffer[15+count] = diff>>8;
		buffer[16+count] = diff;
		count += 5;
	} else {
		//debug1("evt time NOT valid");
		buffer[11] = amid;
		memcpy(&buffer[12], comms_get_payload(iface, msg, count), count);
	}

	buffer[0] = 11 + count + 2; // hdr, data, crc
	total = 1 + 11 + count + 2; // lenb, hdr, data, crc
	//RAIL_WriteTxFifo(radio_rail_handle, buffer, count, true);
	RAIL_SetTxFifo(radio_rail_handle, buffer, total, sizeof(buffer));
	radio_send_time = radio_sent_time = RAIL_GetTime();
	// if ack is required in FCF
	if(radio_tx_wait_ack) {
		rslt = RAIL_StartCcaCsmaTx(radio_rail_handle, radio_channel, RAIL_TX_OPTION_WAIT_FOR_ACK, &csmaConf, NULL);
	} else {
		rslt = RAIL_StartCcaCsmaTx(radio_rail_handle, radio_channel, 0, &csmaConf, NULL);
	}
	debug1("snd %04"PRIX16"->%04"PRIX16"[%02"PRIX8"](%"PRIx8":%"PRIu8")=%d %p l:%d", src, dst, amid, radio_tx_num, radio_send_retries, rslt, msg, total);

	if(rslt == RAIL_STATUS_NO_ERROR) {
		osTimerStart(radio_send_timeout_timer, 1000);
	} else {
		RAIL_Idle(radio_rail_handle, RAIL_IDLE_FORCE_SHUTDOWN, 1);
		RAIL_StartRx(radio_rail_handle, radio_channel, NULL);
		radio_send_fail = true;
	}

	osMutexRelease(radio_mutex);
}

static void radio_resend_timeout_callback(void* argument) {
	uint8_t retu = comms_get_retries_used((comms_layer_t *)&radio_iface, radio_msg_sending->msg) + 1;
	comms_set_retries_used((comms_layer_t *)&radio_iface, radio_msg_sending->msg, retu);
	radio_send_message(radio_msg_sending->msg);
	vPortEnterCritical();
	rx_ack_timeout = false;
	vPortExitCritical();
}

static void radio_send_next() {
	radio_msg_sending = radio_msg_queue_head;
	radio_msg_queue_head = radio_msg_queue_head->next;
	radio_send_retries = 0;
	++radio_tx_num;
	radio_send_message(radio_msg_sending->msg);
}

static void signal_send_done(comms_error_t err) {
	comms_send_done_f *send_done = NULL;
	comms_msg_t* msgp;
	void *user;

	while(osMutexAcquire(radio_mutex, 1000) != osOK);
	osTimerStop(radio_send_timeout_timer);
	radio_send_done_flag = false;
	radio_send_fail = false;
	radio_send_busy = false;
	radio_send_timeout = false;
	rx_ack_timeout = false;

	if(radio_msg_sending != NULL) {
		user = radio_msg_sending->user;
		msgp = radio_msg_sending->msg;
		send_done = radio_msg_sending->send_done;
		radio_msg_sending->next = (radio_queue_element_t*)radio_msg_queue_free;
		radio_msg_queue_free = radio_msg_sending;
		radio_msg_sending = NULL;
	}

	osMutexRelease(radio_mutex);

	if(send_done != NULL) {
		if(err == COMMS_SUCCESS) {
			comms_set_timestamp((comms_layer_t *)&radio_iface, msgp, radio_timestamp());
			_comms_set_ack_received((comms_layer_t *)&radio_iface, msgp);
		}
		logger(err==COMMS_SUCCESS?LOG_INFO1:LOG_WARN1, "snt %p e:%d t:%"PRIu32, msgp, err, radio_sent_time-radio_send_time);
		send_done((comms_layer_t *)&radio_iface, msgp, err, user);
	}
	else err1("snt ? e:%d", err);
}

void radio_run() {
	// If an exception has occurred and RAIL is broken -------------------------
	if(radio_restart == true) {
		while(osMutexAcquire(radio_mutex, 1000) != osOK);
		warn1("restart");
		radio_rail_handle = radio_rail_init();
		if(radio_rail_handle == NULL) {
			__ASM volatile("cpsid i" : : : "memory");
			while(1);
		}
		if(radio_msg_sending != NULL) { // If sending, cancel and notify user
			radio_send_fail = true;
		}
		osMutexRelease(radio_mutex);
	}

	if((radio_msg_sending == NULL)&&(radio_msg_queue_head != NULL)) {
		radio_send_next();
	}

	// Sending has completed ---------------------------------------------------
	if(radio_send_done_flag) {
		if(radio_tx_wait_ack) { // Alternatively we should get rx_ack_timeout
			debug1("ackd %d", rx_ack_timeout);
			comms_ack_received((comms_layer_t *)&radio_iface, radio_msg_sending->msg);
		}
		signal_send_done(COMMS_SUCCESS);
	}

	// Sending has not completed in a reasonable amount of time ----------------
	if(radio_send_timeout) {
		while(osMutexAcquire(radio_mutex, 1000) != osOK);
		RAIL_Idle(radio_rail_handle, RAIL_IDLE_FORCE_SHUTDOWN, 1);
		RAIL_StartRx(radio_rail_handle, radio_channel, NULL);
		osMutexRelease(radio_mutex);

		debug1("TIMEOUT");
		signal_send_done(COMMS_ETIMEOUT);
	}

	// CSMA has failed to transmit the message ---------------------------------
	if(radio_send_busy) {
		bool resend = false;
		while(osMutexAcquire(radio_mutex, 1000) != osOK);
		radio_send_busy = false;
		if(radio_send_retries < 7) {
			resend = true;
			radio_send_retries++;
		}
		osMutexRelease(radio_mutex);
		if(resend) {
			radio_send_message(radio_msg_sending->msg);
		} else {
			signal_send_done(COMMS_EBUSY);
		}
	}

	// Sending has failed ------------------------------------------------------
	if(radio_send_fail) {
		signal_send_done(COMMS_FAIL);
	}

	// RX busy handling -----------------------------------------------------
	if(rx_busy || rx_overflow) {
		uint8_t rxb __attribute__((unused));
		uint8_t rxo __attribute__((unused));
		vPortEnterCritical();
		rxb = rx_busy;
		rxo = rx_overflow;
		rx_busy = 0;
		rx_overflow = 0;
		vPortExitCritical();
		warn1("rx b:%"PRIu8" o:%"PRIu8, rxb, rxo);
	}

	// RX failure handling -----------------------------------------------------
	if((rx_abort > 100) || rx_fail) {
		uint8_t rxa __attribute__((unused));
		uint8_t rxf __attribute__((unused));
		vPortEnterCritical();
		rxa = rx_abort;
		rxf = rx_fail;
		rx_abort = 0;
		rx_fail = 0;
		vPortExitCritical();
		warn1("rx a:%"PRIu8" f:%"PRIu8, rxa, rxf);
	}

	// RX frame error handling -----------------------------------------------------
	if(rx_frame_error) {
		uint8_t rxfe __attribute__((unused));
		vPortEnterCritical();
		rxfe = rx_frame_error;
		rx_frame_error = 0;
		vPortExitCritical();
		warn1("rx fe:%"PRIu8, rxfe);
	}

	// RX ack timeout handling -----------------------------------------------------
	if(rx_ack_timeout) {
		bool resend = false;

		while(osMutexAcquire(radio_mutex, 1000) != osOK);

		if(comms_get_retries_used((comms_layer_t *)&radio_iface, radio_msg_sending->msg) < comms_get_retries((comms_layer_t *)&radio_iface, radio_msg_sending->msg)) {
			resend = true;
		}
		osMutexRelease(radio_mutex);

		logger(resend?LOG_DEBUG1:LOG_WARN1, "rx ackTimeout (%"PRIu8"/%"PRIu8")",
		       comms_get_retries_used((comms_layer_t *)&radio_iface, radio_msg_sending->msg),
		       comms_get_retries((comms_layer_t *)&radio_iface, radio_msg_sending->msg));
		if(resend) {
			radio_send_retries = 0;
			osTimerStart(radio_resend_timer, comms_get_timeout((comms_layer_t *)&radio_iface, radio_msg_sending->msg));
		} else {
			vPortEnterCritical();
			rx_ack_timeout = false;
			vPortExitCritical();
			signal_send_done(COMMS_ENOACK);
		}
	}

	// TX ack sent -----------------------------------------------------------------
	if(tx_ack_sent) {
		uint8_t tas __attribute__((unused));
		vPortEnterCritical();
		tas = tx_ack_sent;
		tx_ack_sent = 0;
		vPortExitCritical();
		info1("tx_ack_sent:%"PRIu8, tas);
	}

	// RX processing -----------------------------------------------------------
	RAIL_RxPacketHandle_t rxh;
	if(osOK == osMessageQueueGet(rxQueue, &rxh, NULL, 0)) {
		RAIL_RxPacketInfo_t packetInfo = {0};
		RAIL_RxPacketHandle_t packetHandle = RAIL_GetRxPacketInfo(radio_rail_handle, rxh, &packetInfo);
		if(packetHandle != RAIL_RX_PACKET_HANDLE_INVALID) {
			RAIL_RxPacketDetails_t packetDetails = {0};
			RAIL_Status_t rx_status = RAIL_GetRxPacketDetailsAlt(radio_rail_handle, packetHandle, &packetDetails);
			if(rx_status == RAIL_STATUS_NO_ERROR) {
				uint8_t buffer[256] = {0};
				RAIL_RxPacketDetails_t timeDetails = packetDetails;
				bool rts_valid = timeDetails.timeReceived.timePosition != RAIL_PACKET_TIME_INVALID;
				uint32_t rts = 0; // timeReceived.packetTime

				if((packetInfo.packetBytes > 11) && (packetInfo.firstPortionData == NULL)) {
					while(1);
				}
				if((packetInfo.packetBytes - packetInfo.firstPortionBytes != 0) && (packetInfo.lastPortionData == NULL)) {
					while(1);
				}
				if(packetInfo.firstPortionBytes > 255) {
					while(1) ;
				}
				if(packetInfo.packetBytes > 255) {
					while(1);
				}
				if(packetInfo.firstPortionBytes > packetInfo.packetBytes) {
					while(1);
				}

				if(rts_valid) {
					// Account for CRC ... unless someone somewhere configures RAIL_RX_OPTION_STORE_CRC?
					timeDetails.timeReceived.totalPacketBytes = packetInfo.packetBytes + 2; // + CRC_BYTES;
					// Want the earliest timestamp possible
					rx_status = RAIL_GetRxTimePreambleStartAlt(radio_rail_handle, &timeDetails);
					if(rx_status == RAIL_STATUS_NO_ERROR) {
						rts = timeDetails.timeReceived.packetTime;
					} else {
						rts_valid = false;
					}
				}

				if(rts_valid == false) {
					warn1("rts %d %d", packetDetails.timeReceived.timePosition, timeDetails.timeReceived.timePosition);
				}

				RAIL_CopyRxPacket(buffer, &packetInfo);

				RAIL_Status_t rst = RAIL_ReleaseRxPacket(radio_rail_handle, packetHandle);
				if(rst != RAIL_STATUS_NO_ERROR) {
					warnb1("rst", &rst, sizeof(RAIL_Status_t));
					while(1);
				}

				uint16_t currTime = (uint16_t)(radio_timestamp() >> 10);
				uint16_t source = ((uint16_t)buffer[8] << 0) | ((uint16_t)buffer[9] << 8);

				if ((!radio_seqNum_save(source, buffer[3], currTime)) && (packetInfo.packetBytes >= 12)) {
					warn1("same seqNum:%02"PRIX8, buffer[3]);
				} else if((packetInfo.packetBytes >= 12) && (buffer[2] == 0x88)
							&& (buffer[5] == 0x00) && (buffer[10] == 0x3F)) {

					comms_msg_t msg;
					am_id_t amid;
					void* payload;
					uint8_t plen;
					uint8_t lqi = 0xFF;
					uint32_t timestamp = radio_timestamp() - (RAIL_GetTime() - rts)/1000;

					comms_init_message((comms_layer_t *)&radio_iface, &msg);
					if(buffer[11] == 0x3D) {
						int32_t diff = (buffer[packetInfo.packetBytes - 4] << 24) |
								   (buffer[packetInfo.packetBytes - 3] << 16) |
								   (buffer[packetInfo.packetBytes - 2] << 8) |
								   (buffer[packetInfo.packetBytes - 1]);

						if((packetInfo.packetBytes < 17) || (packetInfo.packetBytes > 200)) {
							while(1);
						}
						amid = buffer[(packetInfo.packetBytes-5)];
						plen = packetInfo.packetBytes - 17;
						if(rts_valid) {
							comms_set_event_time((comms_layer_t *)&radio_iface, &msg, (uint32_t)(diff + timestamp));
						}
					} else {
						amid = buffer[11];
						plen = packetInfo.packetBytes - 12;
					}

					payload = comms_get_payload((comms_layer_t *)&radio_iface, &msg, plen);

					if(payload != NULL) {
						uint16_t dest = ((uint16_t)buffer[6] << 0) | ((uint16_t)buffer[7] << 8);

						comms_set_packet_type((comms_layer_t *)&radio_iface, &msg, amid);
						comms_set_payload_length((comms_layer_t *)&radio_iface, &msg, plen);
						memcpy(payload, (const void *)&buffer[12], plen);

						if(rts_valid) {
							comms_set_timestamp((comms_layer_t *)&radio_iface, &msg, timestamp);
						}
						_comms_set_rssi((comms_layer_t *)&radio_iface, &msg, packetDetails.rssi);
						if(packetDetails.rssi < -96) {
							lqi = 0;
						} else if(packetDetails.rssi < -93) {
							lqi = lqi + (packetDetails.rssi+93)*50;
						}
						_comms_set_lqi((comms_layer_t *)&radio_iface, &msg, lqi);
						comms_am_set_destination((comms_layer_t *)&radio_iface, &msg, dest);
						comms_am_set_source((comms_layer_t *)&radio_iface, &msg, source);

						debugb1("rx %04"PRIX16"->%04"PRIX16"[%02"PRIX8"] %"PRIu32" r:%"PRIi8" l:%"PRIu8" %"PRIu8":", &(buffer[12]), 8,
						        source, dest, amid,
						        rts,
						        packetDetails.rssi, lqi, plen);
						comms_deliver((comms_layer_t *)&radio_iface, &msg);
					}
					else warn1("rx bad pl %02"PRIX8" %"PRIu8, amid, plen);
				}
				else warnb1("rx bad l=%"PRIu16, buffer, packetInfo.packetBytes > 128 ? 128: packetInfo.packetBytes, packetInfo.packetBytes);
			}
			else err1("rxd");
		}
		else err1("rxi");
	}
}

static void radio_thread(void *p) {
	while(true) {
		radio_run();
	}
}


bool radio_poll() {
	bool busy;

	while(osMutexAcquire(radio_mutex, 1000) != osOK);
	busy = (radio_msg_sending != NULL)||(radio_msg_queue_head != NULL);
	osMutexRelease(radio_mutex);

	return busy;
}


void ackWaitTimer(RAIL_Handle_t cbArg)
{
	if (rx_ack_timeout == 0) {
		comms_ack_received((comms_layer_t *)&radio_iface, radio_msg_sending->msg);
		radio_send_done_flag = true;
	}
}


static void radio_rail_event_cb(RAIL_Handle_t radio_rail_handle, RAIL_Events_t events) {

	if(events & RAIL_EVENTS_TX_COMPLETION) {
		if(events & RAIL_EVENT_TX_PACKET_SENT) {
			radio_sent_time = RAIL_GetTime();
			if(radio_tx_wait_ack) {
				// Wait for the ack or the RAIL_EVENT_RX_ACK_TIMEOUT event
			} else {
				radio_send_done_flag = true;
			}
		} else {
			if(events & RAIL_EVENT_TX_CHANNEL_BUSY) {
				radio_send_busy = true;
			} else { // (RAIL_EVENT_TX_BLOCKED | RAIL_EVENT_TX_ABORTED | RAIL_EVENT_TX_UNDERFLOW)
				radio_send_fail = true;
			}
		}
	}

	if(events & RAIL_EVENTS_RX_COMPLETION) {
		bool unhandled = true;
		if(events & RAIL_EVENT_RX_PACKET_RECEIVED) {
			RAIL_RxPacketHandle_t rxh = RAIL_HoldRxPacket(radio_rail_handle);
			if(rxh != RAIL_RX_PACKET_HANDLE_INVALID) {
				RAIL_RxPacketInfo_t pi;
				if(RAIL_GetRxPacketInfo(radio_rail_handle, rxh, &pi) == rxh) {
					if(pi.packetBytes == 4) { // Inspect if it is an ack
						uint8_t buffer[4];
						RAIL_CopyRxPacket(buffer, &pi);
						if((buffer[0] == 0x05) && (buffer[1] == 0x02)) {
							if(radio_tx_wait_ack) { // Could also check the actual seq, but we assume RAIL does that
								radio_send_done_flag = true;
							}
							RAIL_ReleaseRxPacket(radio_rail_handle, rxh);
							rxh = RAIL_RX_PACKET_HANDLE_INVALID;
						}
					}
				}

				if(rxh != RAIL_RX_PACKET_HANDLE_INVALID) { // Would have been discarded if it was ack
					if(osMessageQueuePut(rxQueue, &rxh, 0, 0) != osOK) {
						RAIL_ReleaseRxPacket(radio_rail_handle, rxh);
						rx_busy++;
					}
				}
			}
			else {
				rx_busy++;
			}
			unhandled = false;
		}
		if(events & RAIL_EVENT_RX_FIFO_OVERFLOW) {
			rx_overflow++;
			unhandled = false;
		}
		if(events & RAIL_EVENT_RX_ADDRESS_FILTERED) {
			// don't care
			unhandled = false;
		}
		if(events & RAIL_EVENT_RX_FRAME_ERROR) {
			rx_frame_error++;
			unhandled = false;
		}
		if(events & RAIL_EVENT_RX_PACKET_ABORTED) {
			rx_abort++;
			unhandled = false;
		}

		if(unhandled) {
			rx_fail++;
		}
	}

	if (events & RAIL_EVENT_TXACK_PACKET_SENT) {
		tx_ack_sent++;
	}

	if(events & RAIL_EVENT_RX_ACK_TIMEOUT) {
		rx_ack_timeout = true;
	}

	if(events & RAIL_EVENT_CAL_NEEDED) {
		//printf("RAIL EVENT CAL NEEDED\n");
		RAIL_Calibrate(radio_rail_handle, NULL, RAIL_CAL_ALL_PENDING);
	}
}

RAIL_AssertErrorCodes_t global_rail_error_code;

void RAILCb_AssertFailed(RAIL_Handle_t railHandle, RAIL_AssertErrorCodes_t errorCode) {
	if(errorCode == RAIL_ASSERT_FAILED_UNEXPECTED_STATE_RX_FIFO) {
		RAIL_Idle(railHandle, RAIL_IDLE, true);
		radio_restart = true;
	} else {
		__ASM volatile("cpsid i" : : : "memory");
		global_rail_error_code = errorCode;
		while(1);
	}
}

static void radio_rail_radio_config_changed_cb(RAIL_Handle_t radio_rail_handle, const RAIL_ChannelConfigEntry_t *entry) {
}

static void radio_rail_rfready_cb(RAIL_Handle_t radio_rail_handle) {
}
