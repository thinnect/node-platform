/**
 * Driver for particle sensor PMS7003
 *
 * @copyright Thinnect
 * @author Konstantin Bilozor
 * @license MIT
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>

#include "em_usart.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "mtimer.h"
#include "pms.h"

#include "loglevels.h"
#define __MODUUL__ "pms"
#define __LOG_LEVEL__ (LOG_LEVEL_pms & BASE_LOG_LEVEL)
#include "log.h"

static uint8_t pmsBuf[PMS_BUF_LEN];
static uint8_t RxBufferIndex = 0;

void USART1_RX_IRQHandler(void)
{
  pmsBuf[RxBufferIndex] = USART_Rx(USART1);

  // Look for PMS packet start
  if (RxBufferIndex != 0) {
    if ((pmsBuf[RxBufferIndex-1] == PMS_START_BYTE_H) &&
      (pmsBuf[RxBufferIndex] == PMS_START_BYTE_L)) {
      pmsBuf[0] = PMS_START_BYTE_H;
      pmsBuf[1] = PMS_START_BYTE_L;
      RxBufferIndex = 1;
    }
  }
  RxBufferIndex++;

  // Wrap once RxBuffer is filled
  if (RxBufferIndex == PMS_BUF_LEN) {
    RxBufferIndex = 0;
  }
}

void init_USART1(void)
{
  CMU_ClockEnable(cmuClock_USART1, true);

  GPIO_PinModeSet(gpioPortA, 5, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortA, 4, gpioModeInputPull, 1);

  // Start with default config, then modify as necessary
  USART_TypeDef           *usart = USART1;
  USART_InitAsync_TypeDef init   = USART_INITASYNC_DEFAULT;
  init.baudrate     = 9600;
  init.enable       = usartDisable;
  USART_InitAsync(USART1, &init);

  // Enable USART pins
  USART1->ROUTEPEN = USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;
  USART1->ROUTELOC0 = (USART1->ROUTELOC0
                      & ~(_USART_ROUTELOC0_TXLOC_MASK
                     | _USART_ROUTELOC0_RXLOC_MASK) )
                     | (USART_ROUTELOC0_TXLOC_LOC5 << _USART_ROUTELOC0_TXLOC_SHIFT)
                     | (USART_ROUTELOC0_RXLOC_LOC3 << _USART_ROUTELOC0_RXLOC_SHIFT);

  USART_IntClear(USART1, USART_IF_RXDATAV);
  USART_IntEnable(USART1, USART_IF_RXDATAV);
  NVIC_ClearPendingIRQ(USART1_RX_IRQn);
  NVIC_EnableIRQ(USART1_RX_IRQn);

  USART_Enable(usart, usartEnable);
}

void pms7003_init() {
	// Set to passive mode command. PMS7003 protocol:
	// | 0x42 | 0x4d | CMD | DATAH | DATAL | CCH | CCL |
	uint8_t set_passive[PMS_CMD_LEN] = {0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70};

	init_USART1();

	for(uint8_t i=0; i<PMS_CMD_LEN; i++) {
		USART_Tx(USART1, set_passive[i]);
	}
	USART_Reset(USART1);
}

bool pms7003_read(uint16_t *pm1_0, uint16_t *pm2_5, uint16_t *pm10) {
	uint16_t sum = 0;
	// Read sensor in passive mode command. PMS7003 protocol:
	// | 0x42 | 0x4d | CMD | DATAH | DATAL | CCH | CCL |
	uint8_t passive_read[PMS_CMD_LEN] = {0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71};
	init_USART1();

	memset(pmsBuf, 0, PMS_BUF_LEN);

	for(uint8_t i=0; i<PMS_CMD_LEN; i++) {
		USART_Tx(USART1, passive_read[i]);
	}

	mtimer_sleep(1000);
	RxBufferIndex = 0;

	for(uint8_t i=0; i<(PMS_BUF_LEN-2); i++) {
		sum += pmsBuf[i];
	}

	debugb1("pmsBuf:", pmsBuf, PMS_BUF_LEN);
	if (sum == 0) {
		warn1("NO PMS DATA");
		return false;
	}

	if (sum == ((pmsBuf[PMS_CC_HIGH] << 8) | pmsBuf[PMS_CC_LOW])) {
		if (pm1_0 != NULL) {
			*pm1_0 = (pmsBuf[PMS_PM1_0_HIGH] << 8) | pmsBuf[PMS_PM1_0_LOW];
			debug1("pm1_0: %"PRIu16, *pm1_0);
		}
		if (pm2_5 != NULL) {
			*pm2_5 = (pmsBuf[PMS_PM2_5_HIGH] << 8) | pmsBuf[PMS_PM2_5_LOW];
			debug1("pm2_5: %"PRIu16, *pm2_5);
		}
		if (pm10 != NULL) {
			*pm10 = (pmsBuf[PMS_PM10_HIGH] << 8) | pmsBuf[PMS_PM10_LOW];
			debug1("pm10: %"PRIu16, *pm10);
		}
		debug1("sum: %04X == %02X%02X", sum, pmsBuf[PMS_CC_HIGH], pmsBuf[PMS_CC_LOW]);
	} else {
		warn1("sum: %04X != %02X%02X", sum, pmsBuf[PMS_CC_HIGH], pmsBuf[PMS_CC_LOW]);
		return false;
	}
	USART_Reset(USART1);
	return true;
}
