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
#include "log.h"


#include "mist_comm_iface.h"
#include "mist_comm_am.h"

#include "cmsis_os2.h"
#include "utils.h"
#include "mist_comm_am.h"

#define MAX_PAYLOAD_SIZE 127

#define BROADCAST_ADDR 0xFFFF

#define MAC_FCF_ACK_REQ_BIT    0x20
#define MAC_FCF_INTRA_PAN_BIT  0x40
#define MAC_FCF_DST_ADDR_BIT   0x0c
#define MAC_FCF_SRC_ADDR_BIT   0xc0

#define MAC_FCF_FRAME_PENDING_BIT 0x10
#define MAC_FCF_FRAME_TYPE     0x07


typedef enum {
	IDLE = 0x0,
	BUSY,
	ERROR,
	RESET
} radio_status_t;

typedef enum {
	NONE = 0x0,
	SEND_FAIL
} radio_errno_t;

/*
#define LL_HW_MODE_STX                  0x00
#define LL_HW_MODE_SRX                  0x01
#define LL_HW_MODE_TRX                  0x02
#define LL_HW_MODE_RTX                  0x03
*/
typedef enum {
	TX_ONLY = 0x0,
	RX_ONLY,
	RX_TX_MODE,
	TX_RX_MODE
} radio_operating_mode_t;

typedef struct {
	uint8_t frame_control[2];
	uint8_t seqnum;
	uint16_t dstPAN;
	uint16_t dst;
	uint16_t src;
	uint8_t data[MAX_PAYLOAD_SIZE];
} __attribute__((packed)) mac_frame_t;


typedef struct {
	uint16_t nodeaddr;
	uint8_t channel;
	uint8_t pan;
	uint8_t status;
	uint8_t mode;
	uint8_t errno;
	osMessageQueueId_t recvQueue;
	osThreadId_t threadid;
	comms_layer_t* radio;
} __attribute__((packed)) radio_config_t;

typedef struct {
	uint8_t id;
	uint8_t data[126];
} __attribute__((packed)) packet_format_t;


extern uint32_t ll_hw_get_tr_mode(void);
extern void  ble_main(void);
extern void hal_rom_code_ini(void);
extern int app_main(void);
extern void init_config(void);
extern  uint32_t pclk;

radio_config_t* init_radio(uint16_t nodeaddr, uint8_t channel, uint8_t pan);
comms_error_t radio_send(comms_layer_iface_t*, comms_msg_t*, comms_send_done_f*, void* user);

#endif
