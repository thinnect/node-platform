/**
* Sensirion SCD30 I2C helper functions
*
* Copyright Thinnect Inc. 2019
* @license MIT
* @author Veiko Rütter
*/

#include "sensirion.h"
#include <stdint.h>
#include <stdio.h>
#include "sensirion_arch_config.h"
#include "sensirion_i2c.h"
#include "retargeti2c.h"

#ifdef USE_CMSIS_OS2
#include "cmsis_os2.h"
#else
#include "mtimer.h"
#endif//USE_CMSIS_OS2

static void delay_ms(uint32_t period)
{
	#ifdef USE_CMSIS_OS2
	osDelay(period);
	#else
	mtimer_sleep(period);
	#endif//USE_CMSIS_OS2
}

int16_t sensirion_i2c_select_bus(uint8_t bus_idx)
{
	return 0;
}

void sensirion_i2c_init(void)
{
}

void sensirion_i2c_release(void)
{
}

int8_t sensirion_i2c_read(uint8_t address, uint8_t *data, uint16_t count)
{
	return(RETARGET_I2CWriteRead(address, NULL, 0, data, count));
}

int8_t sensirion_i2c_write(uint8_t address, const uint8_t *data, uint16_t count)
{
	return(RETARGET_I2CWriteRead(address, (uint8_t *)data, count, NULL, 0));
}

void sensirion_sleep_usec(uint32_t useconds)
{
	delay_ms(useconds / 1000);
}
