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
#include "circular_queue.h"
#include "mist_comm_iface.h"
#include "mist_comm_am.h"

#include "loglevels.h"
#define __MODUUL__ "radio"
#define __LOG_LEVEL__ (LOG_LEVEL_radio & BASE_LOG_LEVEL)
#include "log.h"

uint16_t radio_address;
comms_layer_am_t radio_iface;

static uint8_t radio_sending;

static uint16_t radio_channel;
static uint8_t radio_tx_num;
static RAIL_Handle_t radio_rail_handle;
static Queue_t radio_rx_packet_queue;
static volatile RAIL_RxPacketHandle_t radio_rx_packet_handle;
static void *radio_user;
static comms_msg_t *radio_msg;
static volatile int radio_send_done_flag;
static volatile int radio_send_busy = 0;
static volatile int radio_send_fail = 0;
static volatile int rx_fail = 0;
volatile bool rx_packet_ready = false;

static void radio_rail_event_cb(RAIL_Handle_t radio_rail_handle, RAIL_Events_t events);
static void radio_rail_radio_config_changed_cb(RAIL_Handle_t radio_rail_handle, const RAIL_ChannelConfigEntry_t *entry);
static void radio_rail_rfready_cb(RAIL_Handle_t radio_rail_handle);
static comms_error_t radio_send(comms_layer_iface_t *iface, comms_msg_t *msg, comms_send_done_f *send_done, void *user);
static comms_send_done_f *radio_send_done;

comms_layer_t* radio_init(uint16_t channel, uint16_t pan_id, uint16_t address) {
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

	radio_channel = channel;
	radio_address = address;
	radio_tx_num = 1;
	radio_rx_packet_handle = RAIL_RX_PACKET_HANDLE_INVALID;
	radio_sending = 0;
	radio_send_done_flag = 0;

	radio_rail_handle = RAIL_Init(&rail_config, &radio_rail_rfready_cb);
	if(radio_rail_handle == NULL) {
		// printf("RAIL INIT ERROR\n");
		return(NULL);
	}

	// Put the variables declared above into the appropriate structure
  	//RAIL_TxPowerCurvesConfig_t txPowerCurvesConfig = { curves24Hp, curvesSg, curves24Lp, piecewiseSegments };
  	RAIL_TxPowerCurvesConfigAlt_t txPowerCurvesConfig = RAIL_DECLARE_TX_POWER_CURVES_CONFIG_ALT;

	// In the Silicon Labs implementation, the user is required to save those curves into
	// to be referenced when the conversion functions are called
	RAIL_InitTxPowerCurvesAlt(&txPowerCurvesConfig);

	RAIL_EnablePaCal(1);

	// Declare the structure used to configure the PA
	//RAIL_TxPowerConfig_t txPowerConfig = { RAIL_TX_POWER_MODE_2P4_HP, 1800, 10 };
	RAIL_TxPowerConfig_t txPowerConfig = { RAIL_TX_POWER_MODE_2P4_HP, 3300, 10 };

	if(RAIL_ConfigTxPower(radio_rail_handle, &txPowerConfig) != RAIL_STATUS_NO_ERROR) {
		// printf("TX POWER CONF ERROR\n");
		// Error: The PA could not be initialized due to an improper configuration.
		// Please ensure your configuration is valid for the selected part.
		return(NULL);
	}

	RAIL_TxPower_t power = 170; // 17dBm
	RAIL_GetTxPowerConfig(radio_rail_handle, &txPowerConfig);
	RAIL_TxPowerLevel_t powerLevel = RAIL_ConvertDbmToRaw(radio_rail_handle, txPowerConfig.mode, power);

	RAIL_SetTxPower(radio_rail_handle, powerLevel);

  	// Initialize Radio Calibrations
  	RAIL_ConfigCal(radio_rail_handle, RAIL_CAL_ALL);

	// Load the channel configuration for the generated radio settings
  	//RAIL_ConfigChannels(radio_rail_handle, channelConfigs[0], &radio_rail_radio_config_changed_cb);
  	(void)radio_rail_radio_config_changed_cb;

  	RAIL_Events_t events = RAIL_EVENT_CAL_NEEDED
                 | RAIL_EVENT_RX_ACK_TIMEOUT
                 | RAIL_EVENTS_RX_COMPLETION
				 | RAIL_EVENTS_TX_COMPLETION;

	RAIL_ConfigEvents(radio_rail_handle, RAIL_EVENTS_ALL, events);
	// RAIL_ConfigEvents(radio_rail_handle, RAIL_EVENTS_ALL, RAIL_EVENTS_ALL);

	if(!queueInit(&radio_rx_packet_queue, MAX_QUEUE_LENGTH)) {
		return(NULL);
	}
	RAIL_ConfigData(radio_rail_handle, &data_config);

	RAIL_IEEE802154_Config2p4GHzRadio(radio_rail_handle);
	RAIL_IEEE802154_Init(radio_rail_handle, &ieee802154_config);
	RAIL_IEEE802154_SetPanId(radio_rail_handle, pan_id, 0);
	RAIL_IEEE802154_SetShortAddress(radio_rail_handle, radio_address, 0);

	RAIL_Idle(radio_rail_handle, RAIL_IDLE, 1);
	RAIL_StartRx(radio_rail_handle, radio_channel, NULL);

	comms_am_create((comms_layer_t *)&radio_iface, radio_address, radio_send); //provide signodeid
	return (comms_layer_t *)&radio_iface;
}

void radio_idle() {
	RAIL_Idle(radio_rail_handle, RAIL_IDLE, 1);
}

void radio_reenable() {
	RAIL_StartRx(radio_rail_handle, radio_channel, NULL);
}

void radio_reenable_channel_panid(uint16_t channel, uint16_t pan_id) {
	radio_channel = channel;
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
	dst = comms_am_get_destination((comms_layer_t *)iface, msg);
	buffer[0] = count + 13;
	if(comms_is_ack_required((comms_layer_t *)iface, msg)) {
		buffer[1] = 0x61;
	} else {
		buffer[1] = 0x41;
	}
	buffer[2] = 0x88;
	buffer[3] = radio_tx_num++;
	buffer[4] = 0x22;
	buffer[5] = 0x00;
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
		radio_sending = 1;
		err = COMMS_SUCCESS;
	}else{
		RAIL_Idle(radio_rail_handle, RAIL_IDLE_FORCE_SHUTDOWN, 1);
		RAIL_StartRx(radio_rail_handle, radio_channel, NULL);
		err = COMMS_FAIL;
	}
	return(err);
}

void radio_poll() {
	uint8_t buffer[256];
	comms_send_done_f *send_done;
	comms_msg_t msg, *msgp;
	void *user;
	RAIL_Status_t rx_status;
	RAIL_RxPacketInfo_t packetInfo;
	RAIL_RxPacketDetails_t packetDetails;
	if (rx_fail == 1) {
		rx_fail = 0;
	}

	if(radio_send_busy) {
		radio_send_busy = 0;
		user = radio_user;
		msgp = radio_msg;
		send_done = radio_send_done;
		radio_sending = 0;
		debug1("EBUSY");
		if(send_done != NULL)send_done((comms_layer_t *)&radio_iface, msgp, COMMS_EBUSY, user);
	}
	if(radio_send_fail) {
		radio_send_fail = 0;
		user = radio_user;
		msgp = radio_msg;
		send_done = radio_send_done;
		radio_sending = 0;
		debug1("FAIL");
		if(send_done != NULL)send_done((comms_layer_t *)&radio_iface, msgp, COMMS_FAIL, user);
	}
	if(radio_send_done_flag) {
		radio_send_done_flag = 0;
		user = radio_user;
		msgp = radio_msg;
		send_done = radio_send_done;
		radio_sending = 0;
		_comms_set_ack_received((comms_layer_t *)&radio_iface, msgp);
		//debug1("sdp: %p", send_done);
		if(send_done != NULL) {
			debug1("sD");
			send_done((comms_layer_t *)&radio_iface, msgp, COMMS_SUCCESS, user);
		} else {
			debug1("sD fail");
		}
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

		radio_rx_packet_handle = RAIL_RX_PACKET_HANDLE_INVALID;
		if((packetInfo.packetBytes >= 12) && (buffer[2] == 0x88) && (buffer[5] == 0x00) && (buffer[10] == 0x3F)) {
			am_id_t amid;
			void* payload;
			uint8_t plen;

			comms_init_message((comms_layer_t *)&radio_iface, &msg);
			amid = buffer[11];
			plen = packetInfo.packetBytes - 12;
			payload = comms_get_payload((comms_layer_t *)&radio_iface, &msg, plen);

			if(payload != NULL) {
				comms_set_packet_type((comms_layer_t *)&radio_iface, &msg, amid);
				comms_set_payload_length((comms_layer_t *)&radio_iface, &msg, plen);
				memcpy(payload, (const void *)&buffer[12], plen);

				comms_set_timestamp((comms_layer_t *)&radio_iface, &msg, timestamp);
				_comms_set_rssi((comms_layer_t *)&radio_iface, &msg, packetDetails.rssi);
				_comms_set_lqi((comms_layer_t *)&radio_iface, &msg, 0xFF);
				comms_am_set_destination((comms_layer_t *)&radio_iface, &msg, ((uint16_t)buffer[6] << 0) | ((uint16_t)buffer[7] << 8));
				comms_am_set_source((comms_layer_t *)&radio_iface, &msg, ((uint16_t)buffer[8] << 0) | ((uint16_t)buffer[9] << 8));

				debug1("rx %02"PRIX8" %"PRIu8, amid, plen);
				comms_deliver((comms_layer_t *)&radio_iface, &msg);
			}
			else err1("rx bad pl %02"PRIX8" %"PRIu8, amid, plen);
		}
		else debug1("rx bad");
	}
}

static void radio_rail_event_cb(RAIL_Handle_t radio_rail_handle, RAIL_Events_t events) {

	if(events & RAIL_EVENTS_TX_COMPLETION) {
		if(events & RAIL_EVENT_TX_PACKET_SENT) {
			radio_send_done_flag = 1;
		} else {
			if(events & RAIL_EVENT_TX_CHANNEL_BUSY) {
				radio_send_busy = 1;
			} else {
				radio_send_fail = 1;
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
			rx_fail = 1;
		}
	}

	if(events & RAIL_EVENT_CAL_NEEDED) {
	//	printf("RAIL EVENT CAL NEEDED\n");
		RAIL_Calibrate(radio_rail_handle, NULL, RAIL_CAL_ALL_PENDING);
	}
}

RAIL_AssertErrorCodes_t global_rail_error_code;

void RAILCb_AssertFailed(RAIL_Handle_t railHandle, RAIL_AssertErrorCodes_t errorCode) {
	__ASM volatile("cpsid i" : : : "memory");
	global_rail_error_code = errorCode;
	while(1);
}

static void radio_rail_radio_config_changed_cb(RAIL_Handle_t radio_rail_handle, const RAIL_ChannelConfigEntry_t *entry) {
}

static void radio_rail_rfready_cb(RAIL_Handle_t radio_rail_handle) {
}
