/*
 * SiLabs RETARGET_Serial inspired I2C retargeting solution.
 *
 * Copyright Thinnect Inc.
 * @license MIT
 * @author Raido Pahtma
 */

#include "retargeti2c.h"
#include "retargeti2cconfig.h"

#include "em_gpio.h"
#include "em_cmu.h"
#include "em_i2c.h"

#include "loglevels.h"
#define __MODUUL__ "ri2c"
#define __LOG_LEVEL__ (LOG_LEVEL_retargeti2c & BASE_LOG_LEVEL)
#include "log.h"


void RETARGET_I2CInit() {
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	i2cInit.freq = I2C_FREQ_FAST_MAX;

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(RETARGET_I2C_CLOCK, true);

	GPIO_PinModeSet(RETARGET_I2C_SDA_PORT, RETARGET_I2C_SDA_PIN, gpioModeWiredAndPullUpFilter, 1);
	GPIO_PinModeSet(RETARGET_I2C_SCL_PORT, RETARGET_I2C_SCL_PIN, gpioModeWiredAndPullUpFilter, 1);

	#if defined(GPIO_I2C_ROUTEEN_SCLPEN) // Series 2
		GPIO->I2CROUTE[RETARGET_I2C_INDEX].SDAROUTE =
		                               (RETARGET_I2C_SDA_PORT << _GPIO_I2C_SDAROUTE_PORT_SHIFT)
		                             | (RETARGET_I2C_SDA_PIN << _GPIO_I2C_SDAROUTE_PIN_SHIFT);
		GPIO->I2CROUTE[RETARGET_I2C_INDEX].SCLROUTE =
		                               (RETARGET_I2C_SCL_PORT << _GPIO_I2C_SCLROUTE_PORT_SHIFT)
		                             | (RETARGET_I2C_SCL_PIN << _GPIO_I2C_SCLROUTE_PIN_SHIFT);
		// Enable I2C interface pins
		GPIO->I2CROUTE[RETARGET_I2C_INDEX].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;
	#else
		RETARGET_I2C_DEV->ROUTEPEN = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
		RETARGET_I2C_DEV->ROUTELOC0 = (RETARGET_I2C_SDA_LOC)|(RETARGET_I2C_SCL_LOC);
	#endif

	// I2C power on
	#ifdef RETARGET_I2C_PWR_PORT
		GPIO_PinModeSet(RETARGET_I2C_PWR_PORT, RETARGET_I2C_PWR_PIN, gpioModePushPull, 1);
		GPIO_PinOutSet(RETARGET_I2C_PWR_PORT, RETARGET_I2C_PWR_PIN);
	#endif

	// Initializing the I2C
	I2C_Init(RETARGET_I2C_DEV, &i2cInit);
}

void RETARGET_I2CDeinit() {
	CMU_ClockEnable(RETARGET_I2C_CLOCK, false);
	// Using PA5 (SDA, 5) and PA4 (SCL, 3)
	GPIO_PinModeSet(RETARGET_I2C_SDA_PORT, RETARGET_I2C_SDA_PIN, gpioModeDisabled, 0);
	GPIO_PinModeSet(RETARGET_I2C_SCL_PORT, RETARGET_I2C_SCL_PIN, gpioModeDisabled, 0);

	// I2C power off
	#ifdef RETARGET_I2C_PWR_PORT
		GPIO_PinModeSet(RETARGET_I2C_PWR_PORT, RETARGET_I2C_PWR_PIN, gpioModeDisabled, 0);
	#endif
}

int8_t RETARGET_I2CRead(uint8_t devAddr, uint8_t regAddr, uint8_t *regData, uint16_t count){
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;
	uint32_t timeout = 300000;

	seq.addr = devAddr << 1;
	seq.flags = I2C_FLAG_WRITE_READ;

	seq.buf[0].len  = 1;
	seq.buf[0].data = &regAddr;
	seq.buf[1].len  = count;
	seq.buf[1].data = regData;

	ret = I2C_TransferInit(RETARGET_I2C_DEV, &seq);
	while (ret == i2cTransferInProgress && timeout--) {
		ret = I2C_Transfer(RETARGET_I2C_DEV);
	}

	if ( ret != i2cTransferDone ) {
		return((int) ret);
	}
	return((int) 0);
}

int8_t RETARGET_I2CWrite(uint8_t devAddr, uint8_t regAddr, uint8_t *regData, uint16_t count){
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret;
	uint32_t timeout = 300000;

	seq.addr = devAddr << 1;
	seq.flags = I2C_FLAG_WRITE_WRITE;

	seq.buf[0].len  = 1;
	seq.buf[0].data = &regAddr;
	seq.buf[1].len  = count;
	seq.buf[1].data = regData;

	ret = I2C_TransferInit(RETARGET_I2C_DEV, &seq);
	while (ret == i2cTransferInProgress && timeout--) {
		ret = I2C_Transfer(RETARGET_I2C_DEV);
	}

	if ( ret != i2cTransferDone ) {
		return((int) ret);
	}
	return((int) 0);
}
