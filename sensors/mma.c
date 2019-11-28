/*
 * Really basic driver for the NXP MMA8653FC.
 * https://www.nxp.com/docs/en/data-sheet/MMA8653FC.pdf
 *
 * Currently only supports polling the Landscape/Portrait register.
 *
 * Copyright Thinnect Inc. 2019
 * @author Raido Pahtma
 * @license MIT
 */

#include "retargeti2c.h"

#ifdef USE_CMSIS_OS2
#include "cmsis_os2.h"
#else
#include "mtimer.h"
#endif//USE_CMSIS_OS2

#include "loglevels.h"
#define __MODUUL__ "mma"
#define __LOG_LEVEL__ (LOG_LEVEL_mma & BASE_LOG_LEVEL)
#include "log.h"


uint8_t mma_read_id() {
	uint8_t regv = 0;
	RETARGET_I2CRead(0x1D, 0x0D, &regv, 1);
	debug1("id:%x", regv);
	return regv;
}

uint8_t mma_activate() {
	uint8_t regv;

	// Enable orientation detection
	regv = 0xC0; // |Debounce mode 1|P/L 1|000000|
	RETARGET_I2CWrite(0x1D, 0x11, &regv, 1); // 0x11 PL_CFG register

	regv = 0x01; // Bit 0 - ACTIVE
	RETARGET_I2CWrite(0x1D, 0x2A, &regv, 1); // 0x2A CTRL_REG1 System Control 1 register

	#ifdef USE_CMSIS_OS2
	osDelay(50);
	#else
	mtimer_sleep(50);
	#endif//USE_CMSIS_OS2

	RETARGET_I2CRead(0x1D, 0x0B, &regv, 1); // 0x0B SYSMOD System Mode register
	debug1("s:%x", regv);
	return regv;
}

uint8_t mma_read_orientation() {
	uint8_t regv = 0;

	RETARGET_I2CRead(0x1D, 0x11, &regv, 1);
	if(regv & 0x40) { // P/L detection enabled
		RETARGET_I2CRead(0x1D, 0x10, &regv, 1); // 0x10 PL_STATUS Register
		debug1("o:%x", regv);
	}
	else {
		warn1("st:%x", regv);
		return 0xFF;
	}
	return regv & 0x7; // orientation bits
}
