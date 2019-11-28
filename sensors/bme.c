/*
 * Really basic wrapper around the Bosch BME280 driver and the RETARGET_I2C
 * component.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Veiko Rütter, Raido Pahtma
 */
#include "bme280_defs.h"
#include "bme280.h"

#include "bme.h"
#include "retargeti2c.h"

#ifdef USE_CMSIS_OS2
#include "cmsis_os2.h"
#else
#include "mtimer.h"
#endif//USE_CMSIS_OS2

#include "loglevels.h"
#define __MODUUL__ "bme"
#define __LOG_LEVEL__ (LOG_LEVEL_bme & BASE_LOG_LEVEL)
#include "log.h"

static void delay_ms(uint32_t period){
	#ifdef USE_CMSIS_OS2
	osDelay(period);
	#else
	mtimer_sleep(period);
	#endif//USE_CMSIS_OS2
}

bool bme280_read(int32_t *temperature, uint32_t *pressure, uint32_t *humidity){
	struct bme280_dev bme280;
	uint8_t bme280_Id;
	int8_t result;
	struct bme280_data comp_data;

	bme280.dev_id = BME280_I2C_ADDR_PRIM;
	bme280.chip_id = BME280_CHIP_ID;
	bme280.intf = BME280_I2C_INTF;
	bme280.read = RETARGET_I2CRead;
	bme280.write = RETARGET_I2CWrite;
	bme280.delay_ms = delay_ms;

	delay_ms(500);

	RETARGET_I2CRead(bme280.dev_id, BME280_CHIP_ID_ADDR, &bme280_Id, 1);
	if(bme280_Id != BME280_CHIP_ID){
		debug1("CHIP ID ERROR");
		return false;
	}

	result = bme280_init(&bme280);
	debug1("INIT: %d", (int)result);

	bme280.settings.osr_h = BME280_OVERSAMPLING_1X;
	bme280.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme280.settings.osr_t = BME280_OVERSAMPLING_2X;
	bme280.settings.filter = BME280_FILTER_COEFF_OFF;
	bme280.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	result = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_STANDBY_SEL | BME280_FILTER_SEL, &bme280);
	debug1("SET SENSOR SETTINGS: %u", (int)result);

	result = bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280);
	debug1("SET SENSOR MODE: %u", (int)result);

	delay_ms(80);

	result = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280);
	debug1("GET SENSOR DATA: %u", (int)result);

	if(temperature != NULL){
		*temperature = comp_data.temperature;
	}
	if(pressure != NULL){
		*pressure = comp_data.pressure;
	}
	if(humidity != NULL){
		*humidity = comp_data.humidity;
	}

	return result >= 0;
}
