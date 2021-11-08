/**
 * Radio driver for h1002.
 *
 * Copyright Thinnect Inc. 2021
 * @license MIT
*/
#ifndef _RADIO_H_
#define _RADIO_H_


#include <stdint.h>
#include <string.h>

#include "rf_phy_driver.h"
#include "global_config.h"
#include "jump_function.h"
#include "mcu.h"
#include "bus_dev.h"

#include "ll_debug.h"
#include "ll_hw_drv.h"
#include "ll_sleep.h"

#include "gpio.h"
#include "pwrmgr.h"


#include "mist_comm_iface.h"
#include "mist_comm_am.h"

#include "cmsis_os2.h"
#include "mist_comm_am.h"

#define MAX_PAYLOAD_SIZE 127

#define BROADCAST_ADDR 0xFFFF

#define MAC_FCF_ACK_REQ_BIT    0x20
#define MAC_FCF_INTRA_PAN_BIT  0x40
#define MAC_FCF_DST_ADDR_BIT   0x0c
#define MAC_FCF_SRC_ADDR_BIT   0xc0

#define MAC_FCF_FRAME_PENDING_BIT 0x10
#define MAC_FCF_FRAME_TYPE     0x07
#define MAC_FCF_ACK_PACKET 0x02

typedef enum
{
    PHY_CCA_IDLE       = 0x00,
    PHY_CCA_BUSY       = 0x01,
} phy_sts_t;

typedef enum
{
	IDLE = 0x0,
	BUSY,
	ERROR,
	RESET
} radio_status_t;

typedef enum
{
	NONE = 0x0,
	SEND_FAIL
} radio_errno_t;

typedef enum
{
	RFPHY_IDLE = 0x0,
	RFPHY_RX_ONLY,
	RFPHY_TX_ONLY,
	RFPHY_TX_RXACK
} radio_operating_mode_t;

typedef struct
{
	uint8_t frame_control[2];
	uint8_t seqnum;
	uint16_t dstPAN;
	uint16_t dst;
	uint16_t src;
	uint8_t data[MAX_PAYLOAD_SIZE];
} __attribute__((packed)) mac_frame_t;

typedef struct
{
	uint8_t len;
	uint8_t fcf[2];
	uint8_t tx_num;
} __attribute__((packed)) ack_pack_t;

typedef struct
{
	int16_t rssi;
	uint8_t buffer[140];
} __attribute__((packed)) data_pckt_t;


typedef struct
{
	uint16_t nodeaddr;
	uint8_t channel;
	uint8_t pan;
	uint8_t status;
	uint8_t mode;
	uint8_t errno;
	int8_t cca_treshhold;
	osMessageQueueId_t recvQueue;
	osThreadId_t threadid;
	comms_layer_t* radio;
} __attribute__((packed)) radio_config_t;

typedef struct
{
	uint8_t id;
	uint8_t data[126];
} __attribute__((packed)) packet_format_t;


extern uint32_t ll_hw_get_tr_mode(void);
extern void  ble_main(void);
extern void hal_rom_code_ini(void);
extern int app_main(void);
extern void init_config(void);
extern  uint32_t pclk;

radio_config_t* init_radio (uint16_t nodeaddr, uint8_t channel, uint8_t pan) __attribute__((section("_section_xip_code_")));
comms_error_t radio_send(comms_layer_iface_t*, comms_msg_t*, comms_send_done_f*, void* user);
/**
 * Radio statistics - count number of transmitted packets (including retries).
 * @return number of transmitted packets.
 */
uint32_t radio_tx_packets();

/**
 * Radio statistics - roughly estimate the number of transmitted bytes.
 * @return Transmitted bytes.
 */
uint32_t radio_tx_bytes();

#endif
