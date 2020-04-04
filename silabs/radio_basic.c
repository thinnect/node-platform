/*
 * Mist-comm compatible SiLabs RAIL based radio layer.
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
#include "rail_ieee802154.h"
#include "rail_types.h"
#include "rail_chip_specific.h"
#include "rail_assert_error_codes.h"
#include "pa_conversions_efr32.h"
#include "pa_curves_efr32.h"
#include "mist_comm_iface.h"
#include "mist_comm_am.h"

#include "radio_seqNum.h"

#include "loglevels.h"
#define __MODUUL__ "radio"
#define __LOG_LEVEL__ (LOG_LEVEL_radio & BASE_LOG_LEVEL)
#include "log.h"

uint16_t radio_address;
static uint16_t radio_pan_id;
static uint8_t radio_channel;
comms_layer_am_t radio_iface;

static bool radio_sending;

static uint8_t radio_tx_num;
static RAIL_Handle_t radio_rail_handle;
volatile bool rx_packet_ready = false;

static volatile RAIL_RxPacketHandle_t radio_rx_packet_handle;
static void *radio_user;
static comms_msg_t *radio_msg;

static volatile bool radio_send_done_flag;
static volatile bool radio_send_busy;
static volatile bool radio_send_fail;
static volatile uint8_t rx_fail;
static volatile bool radio_restart;

static RAIL_Handle_t radio_rail_init();
static void radio_rail_event_cb(RAIL_Handle_t radio_rail_handle, RAIL_Events_t events);
static void radio_rail_radio_config_changed_cb(RAIL_Handle_t radio_rail_handle, const RAIL_ChannelConfigEntry_t *entry);
static void radio_rail_rfready_cb(RAIL_Handle_t radio_rail_handle);
static comms_error_t radio_send(comms_layer_iface_t *iface, comms_msg_t *msg, comms_send_done_f *send_done, void *user);
static comms_send_done_f *radio_send_done;

comms_layer_t* radio_init(uint8_t channel, uint16_t pan_id, uint16_t address) {
	radio_channel = channel;
	radio_pan_id = pan_id;
	radio_address = address;
	radio_tx_num = 0;

	radio_rail_handle = radio_rail_init();
	if(radio_rail_handle != NULL) {
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
			.ackTimeout = 892,
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
	rx_fail = 0;

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

	RAIL_TxPower_t power = DEFAULT_RFPOWER_DBM * 10; // RAIL uses deci-dBm
	RAIL_GetTxPowerConfig(radio_rail_handle, &txPowerConfig);
	RAIL_TxPowerLevel_t powerLevel = RAIL_ConvertDbmToRaw(radio_rail_handle, txPowerConfig.mode, power);

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
	                     ;
	//           | RAIL_EVENTS_RX_COMPLETION

	RAIL_ConfigEvents(handle, RAIL_EVENTS_ALL, events);
	// RAIL_ConfigEvents(handle, RAIL_EVENTS_ALL, RAIL_EVENTS_ALL);

	radio_sending = false;
	RAIL_ConfigData(handle, &data_config);

	RAIL_IEEE802154_Config2p4GHzRadio(handle);
	RAIL_IEEE802154_Init(handle, &ieee802154_config);
	RAIL_IEEE802154_SetPanId(handle, radio_pan_id, 0);
	RAIL_IEEE802154_SetShortAddress(handle, radio_address, 0);

	RAIL_Idle(handle, RAIL_IDLE, 1);
	if(RAIL_StartRx(handle, radio_channel, NULL) != RAIL_STATUS_NO_ERROR) {
		err1("StartRx");
	}
	return handle;
}

void radio_idle() {
	RAIL_Idle(radio_rail_handle, RAIL_IDLE, 1);
}

void radio_reenable() {
	RAIL_StartRx(radio_rail_handle, radio_channel, NULL);
}

void radio_reenable_channel_panid(uint8_t channel, uint16_t pan_id) {
	radio_channel = channel;
	radio_pan_id = pan_id;
	RAIL_IEEE802154_SetPanId(radio_rail_handle, pan_id, 0);
	RAIL_StartRx(radio_rail_handle, radio_channel, NULL);
}

static comms_error_t radio_send(comms_layer_iface_t *iface, comms_msg_t *msg, comms_send_done_f *send_done, void *user) {
	comms_error_t err;
	static uint8_t buffer[256];
	int e, count;
	uint16_t src, dst;
	RAIL_CsmaConfig_t csmaConf = {3, 5, 5, -75, 320, 128, 0};
	if(iface != (comms_layer_iface_t *)&radio_iface)return(COMMS_EINVAL);
	radio_msg = msg;
	radio_send_done = send_done;
	radio_user = user;
	count = comms_get_payload_length((comms_layer_t *)iface, msg);
	src = comms_am_get_source((comms_layer_t *)iface, msg);
	if(src == 0) {
		src = radio_address;
	}
	dst = comms_am_get_destination((comms_layer_t *)iface, msg);
	if(dst == 0) {
		warn1("dest not set");
	}
	buffer[0] = count + 13;
	if(comms_is_ack_required((comms_layer_t *)iface, msg)) {
		buffer[1] = 0x61;
	} else {
		buffer[1] = 0x41;
	}
	buffer[2] = 0x88;
	buffer[3] = radio_tx_num++;
	buffer[4] = ((radio_pan_id >> 0) & 0xFF);
	buffer[5] = ((radio_pan_id >> 8) & 0xFF);
	buffer[6] = ((dst >> 0) & 0xFF);
	buffer[7] = ((dst >> 8) & 0xFF);
	buffer[8] = ((src >> 0) & 0xFF);
	buffer[9] = ((src >> 8) & 0xFF);
	buffer[10] = 0x3F;
	buffer[11] = comms_get_packet_type((comms_layer_t *)iface, msg);
	memcpy(&buffer[12], comms_get_payload((comms_layer_t *)iface, msg, count), count);

	RAIL_SetTxFifo(radio_rail_handle, buffer, count + 14, 256);
  	e = RAIL_StartCcaCsmaTx(radio_rail_handle, radio_channel, 0, &csmaConf, NULL);
	debug1("snd e: %i", e);
	if(e == RAIL_STATUS_NO_ERROR) {
		radio_sending = true;
		err = COMMS_SUCCESS;
	}else{
		RAIL_Idle(radio_rail_handle, RAIL_IDLE_FORCE_SHUTDOWN, 1);
		RAIL_StartRx(radio_rail_handle, radio_channel, NULL);
		err = COMMS_FAIL;
	}
	return(err);
}

bool radio_poll() {
	uint8_t buffer[256];
	comms_send_done_f *send_done;
	comms_msg_t msg, *msgp;
	void *user;
	RAIL_Status_t rx_status;
	RAIL_RxPacketInfo_t packetInfo;
	RAIL_RxPacketDetails_t packetDetails;
	bool active = false; // Set to true if poll does some actual work

	if(radio_restart == true) {
		warn1("restart");
		radio_rail_handle = radio_rail_init();
		if(radio_rail_handle == NULL) {
			__ASM volatile("cpsid i" : : : "memory");
			while(1);
		}
		if(radio_sending) { // If sending, cancel and notify user
			radio_send_fail = true;
		}
		active = true;
	}

	if (rx_fail == 1) {
		rx_fail = 0;
	}

	if(radio_send_busy) {
		radio_send_busy = 0;
		user = radio_user;
		msgp = radio_msg;
		send_done = radio_send_done;
		radio_sending = false;
		debug1("EBUSY");
		if(send_done != NULL)send_done((comms_layer_t *)&radio_iface, msgp, COMMS_EBUSY, user);
		active = true;
	}
	if(radio_send_fail) {
		radio_send_fail = 0;
		user = radio_user;
		msgp = radio_msg;
		send_done = radio_send_done;
		radio_sending = false;
		debug1("FAIL");
		if(send_done != NULL)send_done((comms_layer_t *)&radio_iface, msgp, COMMS_FAIL, user);
		active = true;
	}
	if(radio_send_done_flag) {
		radio_send_done_flag = 0;
		user = radio_user;
		msgp = radio_msg;
		send_done = radio_send_done;
		radio_sending = false;
		_comms_set_ack_received((comms_layer_t *)&radio_iface, msgp);
		//debug1("sdp: %p", send_done);
		if(send_done != NULL) {
			debug1("sD");
			send_done((comms_layer_t *)&radio_iface, msgp, COMMS_SUCCESS, user);
		} else {
			debug1("sD fail");
		}
		active = true;
	}
	if(radio_rx_packet_handle != RAIL_RX_PACKET_HANDLE_INVALID) {
		uint32_t timestamp = 0;
		if (rx_packet_ready == true) {
			// WHILE(true) loops are used ONLY for development purposes
			if (RAIL_GetRxPacketInfo(radio_rail_handle, radio_rx_packet_handle, &packetInfo) == RAIL_RX_PACKET_HANDLE_INVALID) {
				while (true) ;
			}

			if ((packetInfo.packetBytes > 11) && (packetInfo.firstPortionData == NULL)) {
				while(true) ;
			}
			if ((packetInfo.packetBytes - packetInfo.firstPortionBytes != 0) && (packetInfo.lastPortionData == NULL)) {
				while(true) ;
			}
			if (packetInfo.firstPortionBytes > 255) {
				while (true) ;
			}
			if (packetInfo.packetBytes > 255) {
				while(true) ;
			}
			if (packetInfo.firstPortionBytes > packetInfo.packetBytes) {
				while(true) ;
			}
			RAIL_CopyRxPacket(buffer, &packetInfo);
			if (radio_rail_handle == NULL) {
				while(true) ;
			}
			if (radio_rx_packet_handle == RAIL_RX_PACKET_HANDLE_INVALID) {
				while(true) ;
			}
			rx_status = RAIL_GetRxPacketDetails(radio_rail_handle, radio_rx_packet_handle, &packetDetails);
			if (rx_status != RAIL_STATUS_NO_ERROR) {
				debug1("rx_status: %d", rx_status);
				while(true) ;
			}
			if (RAIL_ReleaseRxPacket(radio_rail_handle, radio_rx_packet_handle) != RAIL_STATUS_NO_ERROR) {
				while (true) ;
			}
			rx_packet_ready = false;
		}

		uint16_t source = ((uint16_t)buffer[8] << 0) | ((uint16_t)buffer[9] << 8);
		uint16_t currTime = (uint16_t)(RAIL_GetTime() / 1000000);

		radio_rx_packet_handle = RAIL_RX_PACKET_HANDLE_INVALID;
		if ((!radio_seqNum_save(source, buffer[3], currTime)) && (packetInfo.packetBytes >= 12)) {
			warn1("same seqNum:%02"PRIX8, buffer[3]);
		} else if((packetInfo.packetBytes >= 12) && (buffer[2] == 0x88) && (buffer[5] == 0x00) && (buffer[10] == 0x3F)) {
			am_id_t amid;
			void* payload;
			uint8_t plen;

			comms_init_message((comms_layer_t *)&radio_iface, &msg);
			amid = buffer[11];
			plen = packetInfo.packetBytes - 12;
			payload = comms_get_payload((comms_layer_t *)&radio_iface, &msg, plen);

			if(payload != NULL) {
				uint16_t dest = ((uint16_t)buffer[6] << 0) | ((uint16_t)buffer[7] << 8);
				comms_set_packet_type((comms_layer_t *)&radio_iface, &msg, amid);
				comms_set_payload_length((comms_layer_t *)&radio_iface, &msg, plen);
				memcpy(payload, (const void *)&buffer[12], plen);

				comms_set_timestamp((comms_layer_t *)&radio_iface, &msg, timestamp);
				_comms_set_rssi((comms_layer_t *)&radio_iface, &msg, packetDetails.rssi);
				_comms_set_lqi((comms_layer_t *)&radio_iface, &msg, 0xFF);
				comms_am_set_destination((comms_layer_t *)&radio_iface, &msg, dest);
				comms_am_set_source((comms_layer_t *)&radio_iface, &msg, source);

				debug1("rx %02"PRIX8" %"PRIu8, amid, plen);
				comms_deliver((comms_layer_t *)&radio_iface, &msg);
				active = true; // We just delivered a message
			}
			else err1("rx bad pl %02"PRIX8" %"PRIu8, amid, plen);
		}
		else debug1("rx bad");
	}
	return active;
}

static void radio_rail_event_cb(RAIL_Handle_t radio_rail_handle, RAIL_Events_t events) {

	if(events & RAIL_EVENTS_TX_COMPLETION) {
		if(events & RAIL_EVENT_TX_PACKET_SENT) {
			radio_send_done_flag = true;
		} else {
			if(events & RAIL_EVENT_TX_CHANNEL_BUSY) {
				radio_send_busy = true;
			} else { // (RAIL_EVENT_TX_BLOCKED | RAIL_EVENT_TX_ABORTED | RAIL_EVENT_TX_UNDERFLOW)
				radio_send_fail = true;
			}
		}
	}

	if ( events & RAIL_EVENTS_RX_COMPLETION ) {
		if ( events & RAIL_EVENT_RX_PACKET_RECEIVED ) {
			if((radio_rx_packet_handle == RAIL_RX_PACKET_HANDLE_INVALID) && (rx_packet_ready == false)) {
				rx_packet_ready = true;
				radio_rx_packet_handle = RAIL_HoldRxPacket(radio_rail_handle);
			}
		} else {
			rx_fail++;
		}
	}

	if(events & RAIL_EVENT_CAL_NEEDED) {
	//	printf("RAIL EVENT CAL NEEDED\n");
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
